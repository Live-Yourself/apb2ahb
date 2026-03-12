//==================================================================
// APB2AHB Bridge - Top Module
// 功能：将APB4总线转换为AHB-Lite总线
// 特性：
//   1. 支持32位地址/数据总线
//   2. 支持等待状态插入
//   3. 支持错误响应
//   4. 支持8/16/32位传输
//   5. 支持4个深度的写缓冲
//==================================================================

module apb2ahb_top (
    // Global signals
    input  wire         PCLK,      // APB时钟
    input  wire         PRESETn,   // APB复位（低有效）
    input  wire         HCLK,      // AHB时钟
    input  wire         HRESETn,   // AHB复位（低有效）
    
    // APB Slave Interface (APB4协议)
    input  wire         PSEL,      // 外设选择
    input  wire         PENABLE,   // 使能信号
    input  wire [31:0]  PADDR,     // 地址总线
    input  wire         PWRITE,    // 读写控制：1=写，0=读
    input  wire [31:0]  PWDATA,    // 写数据
    input  wire [3:0]   PSTRB,     // 字节使能（写掩码）
    output reg  [31:0]  PRDATA,    // 读数据
    output wire         PREADY,    // 传输完成
    output wire         PSLVERR,   // 错误响应
    
    // AHB Master Interface (AHB-Lite协议)
    output reg  [31:0]  HADDR,     // 地址总线
    output reg  [2:0]   HBURST,    // 突发类型
    output reg          HMASTLOCK, // 锁定传输
    output reg  [3:0]   HPROT,     // 保护控制
    output reg  [2:0]   HSIZE,     // 传输大小
    output reg  [1:0]   HTRANS,    // 传输类型
    output reg          HWRITE,    // 读写控制
    output reg  [31:0]  HWDATA,    // 写数据
    input  wire [31:0]  HRDATA,    // 读数据
    input  wire         HREADY,    // 传输完成
    input  wire [1:0]   HRESP,     // 响应类型
    
    // Configuration Interface
    input  wire         BRIDGE_EN, // 桥接器使能
    output wire         BUSY,      // 忙标志
    output wire         ERROR      // 错误标志
);

//==================================================================
// 内部信号声明
//==================================================================

// 时钟域交叉同步信号
wire        apb_valid;      // APB传输有效
wire        ahb_ready;      // AHB传输完成
wire        ahb_error;      // AHB错误响应

// 控制信号
wire        transfer_start; // 传输开始
wire        transfer_done;  // 传输完成
wire        write_buffer_full;  // 写缓冲满
wire        write_buffer_empty; // 写缓冲空

// 数据通路信号
wire [31:0] apb_addr_sync;  // 同步后的APB地址
wire [31:0] apb_wdata_sync; // 同步后的APB写数据
wire [31:0] ahb_rdata_sync; // 同步后的AHB读数据

// 配置寄存器信号
wire [31:0] config_reg;     // 配置寄存器
wire [31:0] status_reg;     // 状态寄存器
wire [31:0] addr_base_reg;  // 基地址寄存器
wire [31:0] addr_mask_reg;  // 地址掩码寄存器

//==================================================================
// 实例化子模块
//==================================================================

// APB从机接口
apb_slave_interface u_apb_slave_if (
    // APB接口
    .PCLK       (PCLK),
    .PRESETn    (PRESETn),
    .PSEL       (PSEL),
    .PENABLE    (PENABLE),
    .PADDR      (PADDR),
    .PWRITE     (PWRITE),
    .PWDATA     (PWDATA),
    .PSTRB      (PSTRB),
    .PRDATA     (PRDATA),
    .PREADY     (PREADY),
    .PSLVERR    (PSLVERR),
    
    // 内部接口
    .transfer_valid (apb_valid),
    .transfer_addr  (apb_addr_sync),
    .transfer_wdata (apb_wdata_sync),
    .transfer_write (PWRITE),
    .transfer_strb  (PSTRB),
    .transfer_ready (transfer_done),
    .transfer_error (ahb_error),
    .transfer_rdata (ahb_rdata_sync),
    
    // 配置接口
    .bridge_enable  (BRIDGE_EN),
    .config_reg     (config_reg),
    .status_reg     (status_reg)
);

// AHB主机接口
ahb_master_interface u_ahb_master_if (
    // AHB接口
    .HCLK       (HCLK),
    .HRESETn    (HRESETn),
    .HADDR      (HADDR),
    .HBURST     (HBURST),
    .HMASTLOCK  (HMASTLOCK),
    .HPROT      (HPROT),
    .HSIZE      (HSIZE),
    .HTRANS     (HTRANS),
    .HWRITE     (HWRITE),
    .HWDATA     (HWDATA),
    .HRDATA     (HRDATA),
    .HREADY     (HREADY),
    .HRESP      (HRESP),
    
    // 内部接口
    .transfer_start  (transfer_start),
    .transfer_addr   (apb_addr_sync),
    .transfer_wdata  (apb_wdata_sync),
    .transfer_write  (PWRITE),
    .transfer_size   (HSIZE),
    .transfer_done   (transfer_done),
    .transfer_error  (ahb_error),
    .transfer_rdata  (ahb_rdata_sync),
    
    // 控制状态
    .busy            (BUSY),
    .error           (ERROR)
);

// 配置寄存器组
reg_file u_reg_file (
    // 时钟复位
    .PCLK       (PCLK),
    .PRESETn    (PRESETn),
    .HCLK       (HCLK),
    .HRESETn    (HRESETn),
    
    // APB接口
    .apb_psel   (PSEL),
    .apb_penable(PENABLE),
    .apb_pwrite (PWRITE),
    .apb_paddr  (PADDR[7:0]),  // 只使用低8位地址访问寄存器
    .apb_pwdata (PWDATA),
    .apb_prdata (),
    
    // 寄存器输出
    .config_reg     (config_reg),
    .status_reg     (status_reg),
    .addr_base_reg  (addr_base_reg),
    .addr_mask_reg  (addr_mask_reg),
    .fifo_thresh_reg()
);

// 数据缓冲（用于写操作）
data_buffer u_data_buffer (
    // 时钟域
    .wr_clk     (PCLK),
    .rd_clk     (HCLK),
    .wr_rstn    (PRESETn),
    .rd_rstn    (HRESETn),
    
    // 写接口（APB侧）
    .wr_en      (apb_valid & PWRITE & ~write_buffer_full),
    .wr_data    ({apb_addr_sync, apb_wdata_sync}),
    .wr_full    (write_buffer_full),
    .wr_almost_full(),
    
    // 读接口（AHB侧）
    .rd_en      (transfer_start & HWRITE & ~write_buffer_empty),
    .rd_data    ({HADDR, HWDATA}),
    .rd_empty   (write_buffer_empty),
    .rd_almost_empty(),
    
    // 状态
    .data_count (),
    .free_count ()
);

//==================================================================
// 时钟域交叉同步逻辑
//==================================================================

// APB到AHB的同步
reg  [31:0] apb_addr_ff1, apb_addr_ff2;
reg  [31:0] apb_wdata_ff1, apb_wdata_ff2;
reg         apb_valid_ff1, apb_valid_ff2;

always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        apb_addr_ff1  <= 32'h0;
        apb_addr_ff2  <= 32'h0;
        apb_wdata_ff1 <= 32'h0;
        apb_wdata_ff2 <= 32'h0;
        apb_valid_ff1 <= 1'b0;
        apb_valid_ff2 <= 1'b0;
    end else begin
        apb_addr_ff1  <= PADDR;
        apb_addr_ff2  <= apb_addr_ff1;
        apb_wdata_ff1 <= PWDATA;
        apb_wdata_ff2 <= apb_wdata_ff1;
        apb_valid_ff1 <= apb_valid;
        apb_valid_ff2 <= apb_valid_ff1;
    end
end

assign apb_addr_sync  = apb_addr_ff2;
assign apb_wdata_sync = apb_wdata_ff2;
assign transfer_start = apb_valid_ff2;

// AHB到APB的同步
reg  [31:0] ahb_rdata_ff1, ahb_rdata_ff2;
reg         ahb_ready_ff1, ahb_ready_ff2;
reg         ahb_error_ff1, ahb_error_ff2;

always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        ahb_rdata_ff1 <= 32'h0;
        ahb_rdata_ff2 <= 32'h0;
        ahb_ready_ff1 <= 1'b0;
        ahb_ready_ff2 <= 1'b0;
        ahb_error_ff1 <= 1'b0;
        ahb_error_ff2 <= 1'b0;
    end else begin
        ahb_rdata_ff1 <= HRDATA;
        ahb_rdata_ff2 <= ahb_rdata_ff1;
        ahb_ready_ff1 <= HREADY;
        ahb_ready_ff2 <= ahb_ready_ff1;
        ahb_error_ff1 <= (HRESP != 2'b00);
        ahb_error_ff2 <= ahb_error_ff1;
    end
end

assign ahb_rdata_sync = ahb_rdata_ff2;
assign ahb_ready      = ahb_ready_ff2;
assign ahb_error      = ahb_error_ff2;

//==================================================================
// 地址映射和译码逻辑
//==================================================================

wire        config_access;  // 配置寄存器访问
wire        memory_access;  // 存储器空间访问
wire [31:0] remapped_addr;  // 重映射后的地址

// 地址空间划分：
// 0x0000_0000 - 0x0000_00FF: 配置寄存器空间
// 0x0001_0000 - 0xFFFF_FFFF: 存储器映射空间
assign config_access = (PADDR[31:8] == 24'h0);
assign memory_access = !config_access;

// 地址重映射：应用基地址和掩码
assign remapped_addr = (PADDR & addr_mask_reg) | addr_base_reg;

//==================================================================
// 控制状态机
//==================================================================

// 状态定义
typedef enum logic [2:0] {
    IDLE        = 3'b000,  // 空闲状态
    APB_SETUP   = 3'b001,  // APB建立阶段
    APB_ACCESS  = 3'b010,  // APB访问阶段
    AHB_ADDR    = 3'b011,  // AHB地址相位
    AHB_DATA    = 3'b100,  // AHB数据相位
    ERROR_RESP  = 3'b101,  // 错误响应
    WAIT        = 3'b110   // 等待状态
} bridge_state_t;

bridge_state_t curr_state, next_state;

// 状态寄存器
always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        curr_state <= IDLE;
    end else begin
        curr_state <= next_state;
    end
end

// 下一状态逻辑
always @(*) begin
    case (curr_state)
        IDLE: begin
            if (apb_valid && BRIDGE_EN)
                next_state = APB_SETUP;
            else
                next_state = IDLE;
        end
        
        APB_SETUP: begin
            if (config_access)
                next_state = APB_ACCESS;
            else
                next_state = AHB_ADDR;
        end
        
        APB_ACCESS: begin
            if (PREADY)
                next_state = IDLE;
            else
                next_state = APB_ACCESS;
        end
        
        AHB_ADDR: begin
            if (HREADY)
                next_state = AHB_DATA;
            else
                next_state = AHB_ADDR;
        end
        
        AHB_DATA: begin
            if (HREADY && !HRESP[1])
                next_state = IDLE;
            else if (HRESP[1])
                next_state = ERROR_RESP;
            else
                next_state = AHB_DATA;
        end
        
        ERROR_RESP: begin
            if (HREADY)
                next_state = IDLE;
            else
                next_state = ERROR_RESP;
        end
        
        WAIT: begin
            if (write_buffer_empty)
                next_state = IDLE;
            else
                next_state = WAIT;
        end
        
        default: next_state = IDLE;
    endcase
end

// 输出逻辑
assign BUSY = (curr_state != IDLE);

endmodule