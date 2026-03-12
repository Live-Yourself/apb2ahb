//==================================================================
// Configuration Register File
// 实现APB可访问的配置寄存器
// 寄存器映射：
//   0x00: CONFIG_REG    - 配置寄存器
//   0x04: STATUS_REG    - 状态寄存器
//   0x08: BASE_ADDR_REG - 基地址寄存器
//   0x0C: ADDR_MASK_REG - 地址掩码寄存器
//   0x10: FIFO_TH_REG   - FIFO阈值寄存器
//==================================================================

module reg_file (
    // 时钟复位
    input  wire         PCLK,
    input  wire         PRESETn,
    input  wire         HCLK,
    input  wire         HRESETn,
    
    // APB接口
    input  wire         apb_psel,
    input  wire         apb_penable,
    input  wire         apb_pwrite,
    input  wire [7:0]   apb_paddr,
    input  wire [31:0]  apb_pwdata,
    output reg  [31:0]  apb_prdata,
    
    // 寄存器输出
    output reg  [31:0]  config_reg,
    output reg  [31:0]  status_reg,
    output reg  [31:0]  addr_base_reg,
    output reg  [31:0]  addr_mask_reg,
    output reg  [31:0]  fifo_thresh_reg
);

//==================================================================
// 寄存器定义
//==================================================================

// 配置寄存器 (RW)
// [0]    : 桥接使能 (1=使能, 0=禁用)
// [2:1]  : 等待周期 (00=0周期, 01=1周期, 10=2周期, 11=3周期)
// [5:3]  : 传输大小 (000=8位, 001=16位, 010=32位)
// [7:6]  : 保护级别 (00=用户模式, 01=特权模式)
// [8]    : 缓存使能
// [9]    : 缓冲使能
// [31:10]: 保留

// 状态寄存器 (RO)
// [0]    : 忙标志
// [1]    : 错误标志
// [2]    : FIFO满标志
// [3]    : FIFO空标志
// [7:4]  : FIFO数据计数
// [31:8] : 保留

// 基地址寄存器 (RW)
// [31:0] : 地址重映射基地址

// 地址掩码寄存器 (RW)
// [31:0] : 地址掩码，用于地址对齐

// FIFO阈值寄存器 (RW)
// [3:0]  : 写阈值
// [7:4]  : 读阈值
// [31:8] : 保留

//==================================================================
// APB访问逻辑
//==================================================================

wire        reg_write_en;
wire        reg_read_en;
wire [31:0] reg_rdata;
reg  [31:0] reg_wdata;
reg  [7:0]  reg_addr;

// 生成读写使能
assign reg_write_en = apb_psel & apb_penable & apb_pwrite;
assign reg_read_en  = apb_psel & apb_penable & !apb_pwrite;

// 地址和数据锁存
always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        reg_addr <= 8'h0;
        reg_wdata <= 32'h0;
    end else if (apb_psel) begin
        reg_addr <= apb_paddr;
        if (apb_pwrite) begin
            reg_wdata <= apb_pwdata;
        end
    end
end

//==================================================================
// 寄存器写操作
//==================================================================
always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        config_reg     <= 32'h0000_0001;  // 默认使能桥接器
        addr_base_reg  <= 32'h0000_0000;
        addr_mask_reg  <= 32'hFFFF_0000;  // 高16位用于基地址
        fifo_thresh_reg <= 32'h0000_0303; // 写阈值3，读阈值3
    end else if (reg_write_en) begin
        case (reg_addr[7:0])
            8'h00: config_reg     <= reg_wdata;
            8'h08: addr_base_reg  <= reg_wdata;
            8'h0C: addr_mask_reg  <= reg_wdata;
            8'h10: fifo_thresh_reg <= reg_wdata;
            default: ;  // 忽略未定义地址
        endcase
    end
end

//==================================================================
// 状态寄存器更新（来自AHB域）
//==================================================================
reg  [3:0] fifo_data_count_ahb;
wire       fifo_full_ahb;
wire       fifo_empty_ahb;
wire       bridge_busy_ahb;
wire       bridge_error_ahb;

// 状态寄存器同步（AHB到APB时钟域）
reg [1:0] sync_busy, sync_error;
reg [3:0] sync_data_count;
reg       sync_fifo_full, sync_fifo_empty;

always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        // 这里应该是从其他模块获取的实际状态
        // 为简化，我们使用模拟值
        bridge_busy_ahb <= 1'b0;
        bridge_error_ahb <= 1'b0;
        fifo_data_count_ahb <= 4'h0;
        fifo_full_ahb  <= 1'b0;
        fifo_empty_ahb <= 1'b1;
    end
end

// 同步器
always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        sync_busy       <= 2'b00;
        sync_error      <= 2'b00;
        sync_data_count <= 4'h0;
        sync_fifo_full  <= 1'b0;
        sync_fifo_empty <= 1'b0;
    end else begin
        sync_busy[0]       <= bridge_busy_ahb;
        sync_busy[1]       <= sync_busy[0];
        sync_error[0]      <= bridge_error_ahb;
        sync_error[1]      <= sync_error[0];
        sync_data_count    <= fifo_data_count_ahb;
        sync_fifo_full     <= fifo_full_ahb;
        sync_fifo_empty    <= fifo_empty_ahb;
    end
end

// 状态寄存器（只读）
always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        status_reg <= 32'h0;
    end else begin
        status_reg[0]   <= sync_busy[1];
        status_reg[1]   <= sync_error[1];
        status_reg[2]   <= sync_fifo_full;
        status_reg[3]   <= sync_fifo_empty;
        status_reg[7:4] <= sync_data_count;
        status_reg[31:8] <= 24'h0;
    end
end

//==================================================================
// APB读数据输出
//==================================================================
always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        apb_prdata <= 32'h0;
    end else if (reg_read_en) begin
        case (reg_addr[7:0])
            8'h00: apb_prdata <= config_reg;
            8'h04: apb_prdata <= status_reg;
            8'h08: apb_prdata <= addr_base_reg;
            8'h0C: apb_prdata <= addr_mask_reg;
            8'h10: apb_prdata <= fifo_thresh_reg;
            default: apb_prdata <= 32'hDEAD_BEEF;  // 未定义地址返回特定值
        endcase
    end
end

endmodule