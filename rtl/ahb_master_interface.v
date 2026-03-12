//==================================================================
// AHB Master Interface
// 实现AHB-Lite协议主机接口
// 支持特性：
//   1. 单次传输和增量突发
//   2. 保护控制
//   3. 传输大小配置
//==================================================================

module ahb_master_interface (
    // AHB接口
    input  wire         HCLK,
    input  wire         HRESETn,
    output reg  [31:0]  HADDR,
    output reg  [2:0]   HBURST,
    output reg          HMASTLOCK,
    output reg  [3:0]   HPROT,
    output reg  [2:0]   HSIZE,
    output reg  [1:0]   HTRANS,
    output reg          HWRITE,
    output reg  [31:0]  HWDATA,
    input  wire [31:0]  HRDATA,
    input  wire         HREADY,
    input  wire [1:0]   HRESP,
    
    // 内部接口
    input  wire         transfer_start,
    input  wire [31:0]  transfer_addr,
    input  wire [31:0]  transfer_wdata,
    input  wire         transfer_write,
    input  wire [2:0]   transfer_size,
    output reg          transfer_done,
    output reg          transfer_error,
    output reg  [31:0]  transfer_rdata,
    
    // 控制状态
    output wire         busy,
    output wire         error
);

//==================================================================
// 内部信号定义
//==================================================================

// 状态定义
typedef enum logic [1:0] {
    S_IDLE      = 2'b00,  // 空闲
    S_ADDR      = 2'b01,  // 地址相位
    S_DATA      = 2'b10,  // 数据相位
    S_ERROR     = 2'b11   // 错误处理
} ahb_state_t;

ahb_state_t curr_state, next_state;

// 控制寄存器
reg         addr_phase_done;
reg         data_phase_done;
reg         error_detected;
reg [2:0]   burst_counter;
reg [31:0]  next_addr;

// 配置寄存器
reg [3:0]   prot_config;   // 保护配置
reg         lock_config;   // 锁定配置
reg [2:0]   burst_config;  // 突发配置

//==================================================================
// 状态寄存器
//==================================================================
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        curr_state <= S_IDLE;
    end else begin
        curr_state <= next_state;
    end
end

//==================================================================
// 下一状态逻辑
//==================================================================
always @(*) begin
    case (curr_state)
        S_IDLE: begin
            if (transfer_start)
                next_state = S_ADDR;
            else
                next_state = S_IDLE;
        end
        
        S_ADDR: begin // 地址相位
            if (HREADY)
                next_state = S_DATA;
            else
                next_state = S_ADDR;
        end
        
        S_DATA: begin // 数据相位
            if (HREADY && !HRESP[1]) begin    // HRESP[1]=1表示error，否则为正常
                if (burst_counter > 0)
                    next_state = S_ADDR;  // 突发传输，继续下一个地址
                else
                    next_state = S_IDLE;  // 传输完成
            end else if (HRESP[1]) begin
                next_state = S_ERROR;     // 错误响应
            end else begin
                next_state = S_DATA;      // 等待数据有效
            end
        end
        
        S_ERROR: begin
            if (HREADY)
                next_state = S_IDLE;
            else
                next_state = S_ERROR;
        end
        
        default: next_state = S_IDLE;
    endcase
end

//==================================================================
// 突发传输控制
//==================================================================
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        burst_counter <= 3'b0;
        next_addr     <= 32'h0;
    end else if (curr_state == S_IDLE && transfer_start) begin
        // 开始新的传输
        case (burst_config)
            3'b000: burst_counter <= 3'b0;  // SINGLE
            3'b001: burst_counter <= 3'b3;  // INCR4
            3'b010: burst_counter <= 3'b7;  // INCR8
            3'b011: burst_counter <= 3'b15; // INCR16
            default: burst_counter <= 3'b0; // 默认单次传输
        endcase
        next_addr <= transfer_addr;
    end else if (curr_state == S_DATA && HREADY && burst_counter > 0) begin
        // 突发传输，更新地址
        burst_counter <= burst_counter - 1'b1;
        case (HSIZE)
            3'b000: next_addr <= next_addr + 1;  // Byte
            3'b001: next_addr <= next_addr + 2;  // Halfword
            3'b010: next_addr <= next_addr + 4;  // Word
            default: next_addr <= next_addr + 4;
        endcase
    end
end

//==================================================================
// AHB信号生成
//==================================================================
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        HADDR     <= 32'h0;
        HTRANS    <= 2'b00;  // IDLE
        HWRITE    <= 1'b0;
        HWDATA    <= 32'h0;
        HSIZE     <= 3'b010;  // 默认32位
        HBURST    <= 3'b000;  // 默认单次传输
        HPROT     <= 4'b0011; // 默认非特权数据访问
        HMASTLOCK <= 1'b0;
    end else begin
        case (curr_state)
            S_IDLE: begin
                HTRANS <= 2'b00;  // IDLE
                HWRITE <= 1'b0;
                HWDATA <= 32'h0;
            end
            
            S_ADDR: begin
                HADDR  <= (burst_counter > 0) ? next_addr : transfer_addr;
                HTRANS <= 2'b10;  // NONSEQ
                HWRITE <= transfer_write;
                HSIZE  <= transfer_size;
                HBURST <= burst_config;
                HPROT  <= prot_config;
                HMASTLOCK <= lock_config;
            end
            
            S_DATA: begin
                if (HWRITE && HREADY) begin
                    HWDATA <= transfer_wdata;
                end
                if (burst_counter > 0 && HREADY) begin
                    HTRANS <= 2'b11;  // SEQ（突发传输）
                end else begin
                    HTRANS <= 2'b00;  // IDLE
                end
            end
            
            S_ERROR: begin
                HTRANS <= 2'b00;  // IDLE
            end
        endcase
    end
end

//==================================================================
// 内部信号更新
//==================================================================
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        transfer_done  <= 1'b0;
        transfer_error <= 1'b0;
        transfer_rdata <= 32'h0;
        addr_phase_done <= 1'b0;
        data_phase_done <= 1'b0;
        error_detected  <= 1'b0;
    end else begin
        // 地址相位完成
        addr_phase_done <= (curr_state == S_ADDR) && HREADY;
        
        // 数据相位完成
        if (curr_state == S_DATA && HREADY && !HRESP[1] && burst_counter == 0) begin
            data_phase_done <= 1'b1;
            transfer_done   <= 1'b1;
        end else begin
            data_phase_done <= 1'b0;
            transfer_done   <= 1'b0;
        end
        
        // 错误检测
        if (curr_state == S_DATA && HRESP[1]) begin
            error_detected  <= 1'b1;
            transfer_error  <= 1'b1;
        end else if (curr_state == S_ERROR && HREADY) begin
            error_detected  <= 1'b0;
            transfer_error  <= 1'b0;
        end
        
        // 读数据锁存
        if (curr_state == S_DATA && HREADY && !HWRITE) begin
            transfer_rdata <= HRDATA;
        end
    end
end

//==================================================================
// 状态输出
//==================================================================
assign busy  = (curr_state != S_IDLE);
assign error = error_detected;

//==================================================================
// 配置寄存器（简化版本）
//==================================================================
always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        prot_config  <= 4'b0011;  // 默认：非特权、数据、可缓冲、可缓存
        lock_config  <= 1'b0;     // 默认不锁定
        burst_config <= 3'b000;   // 默认单次传输
    end
    // 实际项目中应从配置寄存器读取
end

endmodule