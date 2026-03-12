//==================================================================
// APB Slave Interface
// 实现APB4协议从机接口
// 支持特性：
//   1. 等待状态插入
//   2. 错误响应生成
//   3. 字节使能处理
//==================================================================

module apb_slave_interface (
    // APB接口
    input  wire         PCLK,
    input  wire         PRESETn,
    input  wire         PSEL,
    input  wire         PENABLE,
    input  wire [31:0]  PADDR,
    input  wire         PWRITE,
    input  wire [31:0]  PWDATA,
    input  wire [3:0]   PSTRB,
    output reg  [31:0]  PRDATA,
    output reg          PREADY,
    output reg          PSLVERR,
    
    // 内部接口
    output wire         transfer_valid,
    output wire [31:0]  transfer_addr,
    output wire [31:0]  transfer_wdata,
    output wire         transfer_write,
    output wire [3:0]   transfer_strb,
    input  wire         transfer_ready,
    input  wire         transfer_error,
    input  wire [31:0]  transfer_rdata,
    
    // 配置接口
    input  wire         bridge_enable,
    input  wire [31:0]  config_reg,
    input  wire [31:0]  status_reg
);

//==================================================================
// 内部信号定义
//==================================================================
reg         psel_ff;      // PSEL寄存器
reg         penable_ff;   // PENABLE寄存器
reg [31:0]  paddr_ff;     // 地址寄存器
reg         pwrite_ff;    // 写控制寄存器
reg [31:0]  pwdata_ff;    // 写数据寄存器
reg [3:0]   pstrb_ff;     // 字节使能寄存器

reg         wait_state;   // 等待状态标志
reg [1:0]   wait_cnt;     // 等待计数器

//==================================================================
// APB输入信号锁存
// 在PSEL有效时锁存输入信号，确保传输期间信号稳定
//==================================================================
always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        psel_ff    <= 1'b0;
        penable_ff <= 1'b0;
        paddr_ff   <= 32'h0;
        pwrite_ff  <= 1'b0;
        pwdata_ff  <= 32'h0;
        pstrb_ff   <= 4'h0;
    end else if (PSEL && !penable_ff) begin
        // 锁存SETUP阶段信号
        psel_ff    <= PSEL;
        penable_ff <= PENABLE;
        paddr_ff   <= PADDR;
        pwrite_ff  <= PWRITE;
        pwdata_ff  <= PWDATA;
        pstrb_ff   <= PSTRB;
    end else if (!PSEL) begin
        // 传输结束，清除锁存信号
        psel_ff    <= 1'b0;
        penable_ff <= 1'b0;
    end
end

//==================================================================
// 传输有效信号生成
// 符合APB协议：PSEL=1 && PENABLE=1
//==================================================================
assign transfer_valid = psel_ff && penable_ff;
assign transfer_addr  = paddr_ff;
assign transfer_wdata = pwdata_ff;
assign transfer_write = pwrite_ff;
assign transfer_strb  = pstrb_ff;

//==================================================================
// 等待状态控制
// 可配置等待周期，模拟外设响应延迟
//==================================================================
reg [1:0] config_wait_cycles;
always @(*) begin
    config_wait_cycles = config_reg[1:0];  // 从配置寄存器读取等待周期
end

always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        wait_state <= 1'b0;
        wait_cnt   <= 2'b0;
    end else if (transfer_valid && !transfer_ready) begin // ready未就绪
        // 进入等待状态
        if (wait_cnt < config_wait_cycles) begin
            wait_state <= 1'b1;
            wait_cnt   <= wait_cnt + 1'b1;
        end else begin
            wait_state <= 1'b0;
            wait_cnt   <= 2'b0;
        end
    end else begin
        wait_state <= 1'b0;
        wait_cnt   <= 2'b0;
    end
end

//==================================================================
// PREADY信号生成
// 组合逻辑：内部传输完成或配置等待周期结束
//==================================================================
always @(*) begin
    if (!bridge_enable) begin
        PREADY = 1'b0;  // 桥接器禁用，不响应
    end else if (transfer_valid) begin
        if (wait_state) begin
            PREADY = 1'b0;  // 等待状态
        end else begin
            PREADY = transfer_ready;  // 内部传输完成
        end
    end else begin
        PREADY = 1'b0;  // 无传输
    end
end

//==================================================================
// PRDATA信号生成
// 在读传输完成时输出读数据
//==================================================================
always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        PRDATA <= 32'h0;
    end else if (transfer_valid && !transfer_write && PREADY) begin
        if (transfer_addr[7:0] < 8'h10) begin
            // 配置寄存器读取
            case (transfer_addr[7:0])
                8'h00: PRDATA <= config_reg;
                8'h04: PRDATA <= status_reg;
                8'h08: PRDATA <= 32'hDEAD_BEEF;  // 设备ID
                8'h0C: PRDATA <= 32'h0000_0100;  // 版本号
                default: PRDATA <= 32'h0;
            endcase
        end else begin
            // 存储器读取
            PRDATA <= transfer_rdata;
        end
    end
end

//==================================================================
// PSLVERR信号生成
// 在错误条件满足时产生错误响应
//==================================================================
always @(*) begin
    if (!bridge_enable) begin
        PSLVERR = 1'b1;  // 桥接器禁用，产生错误
    end else if (transfer_valid) begin
        // 检查地址是否对齐
        if (!transfer_write) begin
            // 读操作：检查地址对齐
            case (transfer_strb)
                4'b0001: PSLVERR = (transfer_addr[1:0] != 2'b00);  // Byte 字节读模式
                4'b0011: PSLVERR = (transfer_addr[0] != 1'b0);     // Halfword 半字读模式
                4'b1111: PSLVERR = (transfer_addr[1:0] != 2'b00);  // Word
                default: PSLVERR = 1'b0;
            endcase
        end else begin
            // 写操作：检查字节使能
            PSLVERR = (transfer_strb == 4'b0000); // transfer_strb==0 代表“没有任何字节被选中”
        end
    end else begin
        PSLVERR = 1'b0;
    end
    
    // 内部错误优先级最高
    if (transfer_error) begin
        PSLVERR = 1'b1;
    end
end

//==================================================================
// 断言（用于形式验证）
//==================================================================

// APB协议检查
property apb_protocol_check;
    @(posedge PCLK) disable iff (!PRESETn)    //disable iff (!PRESETn) 表示复位时禁用这些断言
    (PSEL && !PENABLE) |=> (PSEL && PENABLE);
endproperty

// 地址稳定检查
property addr_stable_check;
    @(posedge PCLK) disable iff (!PRESETn)
    (PSEL) |-> $stable(PADDR);
endproperty

// 写数据稳定检查
property wdata_stable_check;
    @(posedge PCLK) disable iff (!PRESETn)
    (PSEL && PWRITE) |-> $stable(PWDATA);
endproperty

endmodule