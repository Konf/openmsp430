`include "openMSP430_defines.v"

module top_arty (
  input logic CLK100MHZ,

  input logic ck_rst,

  input logic [3:0] sw,
  input logic [3:0] btn,

  // output logic led0_b,
  // output logic led0_g,
  // output logic led0_r,
  // output logic led1_b,
  // output logic led1_g,
  // output logic led1_r,
  // output logic led2_b,
  // output logic led2_g,
  // output logic led2_r,
  // output logic led3_b,
  // output logic led3_g,
  // output logic led3_r,

  output logic [3:0] led,

  // logic [7:0] ja,
  // logic [7:0] jb,
  // logic [7:0] jc,
  // logic [7:0] jd,

  inout ck_io0,
  inout ck_io1,
  inout ck_io2,
  inout ck_io3,
  inout ck_io4,
  inout ck_io5,
  inout ck_io6,
  inout ck_io7,

  input ck_io8,
  output ck_io9,

  output logic uart_rxd_out,
  input  logic uart_txd_in

);









//=============================================================================
// 1)  INTERNAL WIRES/REGISTERS/PARAMETERS DECLARATION
//=============================================================================

// Clock generation
wire               clk_100mhz;
wire               dcm_locked;
wire               dcm_clkfx;
wire               dcm_clk0;
wire               dcm_clkfb;
wire               dco_clk;

// Reset generation
wire               reset_pin;
wire               reset_pin_n;
wire               reset_n;

// Debug interface
wire               omsp_dbg_i2c_scl;
wire 	           omsp_dbg_i2c_sda_in;
wire               omsp_dbg_i2c_sda_out;
wire               omsp0_dbg_i2c_sda_out;
wire               omsp1_dbg_i2c_sda_out;
wire        [23:0] chipscope_trigger;

logic dbg_uart_txd;
logic dbg_uart_rxd;

// Data memory
wire [`DMEM_MSB:0] omsp0_dmem_addr;
wire               omsp0_dmem_cen;
wire               omsp0_dmem_cen_sp;
wire               omsp0_dmem_cen_dp;
wire        [15:0] omsp0_dmem_din;
wire         [1:0] omsp0_dmem_wen;
wire        [15:0] omsp0_dmem_dout;
wire        [15:0] omsp0_dmem_dout_sp;
wire        [15:0] omsp0_dmem_dout_dp;
reg                omsp0_dmem_dout_sel;

wire [`DMEM_MSB:0] omsp1_dmem_addr;
wire               omsp1_dmem_cen;
wire               omsp1_dmem_cen_sp;
wire               omsp1_dmem_cen_dp;
wire        [15:0] omsp1_dmem_din;
wire         [1:0] omsp1_dmem_wen;
wire        [15:0] omsp1_dmem_dout;
wire        [15:0] omsp1_dmem_dout_sp;
wire        [15:0] omsp1_dmem_dout_dp;
reg                omsp1_dmem_dout_sel;

// Program memory
wire [`PMEM_MSB:0] omsp0_pmem_addr;
wire               omsp0_pmem_cen;
wire        [15:0] omsp0_pmem_din;
wire         [1:0] omsp0_pmem_wen;
wire        [15:0] omsp0_pmem_dout;

wire [`PMEM_MSB:0] omsp1_pmem_addr;
wire               omsp1_pmem_cen;
wire        [15:0] omsp1_pmem_din;
wire         [1:0] omsp1_pmem_wen;
wire        [15:0] omsp1_pmem_dout;

// UART
wire               omsp0_uart_rxd;
wire               omsp0_uart_txd;

// LEDs & Switches
wire         [3:0] omsp_switch;
wire         [1:0] omsp0_led;
wire         [1:0] omsp1_led;


logic [7:0] gpio_in;
logic [7:0] gpio_out;
logic [7:0] gpio_dir;

//=============================================================================
// 2)  RESET GENERATION & FPGA STARTUP
//=============================================================================

// Reset input buffer
IBUF   ibuf_reset_n   (.O(reset_pin), .I(~ck_rst));
assign reset_pin_n = ~reset_pin;

// Release the reset only, if the DCM is locked
assign  reset_n = reset_pin_n & dcm_locked;

// Top level reset generation
//wire dco_rst;
//omsp_sync_reset sync_reset_dco (.rst_s (dco_rst), .clk(dco_clk), .rst_a(!reset_n));


//=============================================================================
// 3)  CLOCK GENERATION
//=============================================================================
/*
// Input buffers
//------------------------
IBUFG ibuf_clk_main   (.O(clk_100mhz),    .I(CLK100MHZ));
//IBUFG ibuf_clk_y2     (.O(),             .I(CLOCK_Y2));
//IBUFG ibuf_clk_y3     (.O(),             .I(CLOCK_Y3));
//IBUFG ibuf_clk_bkup   (.O(),             .I(BACKUP_CLK));


// Digital Clock Manager
//------------------------
DCM_SP #(.CLKFX_MULTIPLY(7),
	 .CLKFX_DIVIDE(25),
	 .CLKIN_PERIOD(10.000)) dcm_inst (

// OUTPUTs
    .CLKDV        (),
    .CLKFX        (dcm_clkfx),
    .CLKFX180     (),
    .LOCKED       (dcm_locked),
    .PSDONE       (),

    .STATUS       (),

    .CLK0         (dcm_clk0),
    .CLK180       (),
    .CLK270       (),
    .CLK2X        (),
    .CLK2X180     (),
    .CLK90        (),

// INPUTs
    .CLKFB        (dcm_clkfb),
    .CLKIN        (clk_100mhz),
    .DSSEN        (1'b0),

    .PSCLK        (1'b0),
    .PSEN         (1'b0),
    .PSINCDEC     (1'b0),
    .RST          (reset_pin)
);

BUFG CLK0_BUFG_INST (
    .I(dcm_clk0),
    .O(dcm_clkfb)
);

//synthesis translate_off
defparam dcm_inst.CLKFX_MULTIPLY  = 7;
defparam dcm_inst.CLKFX_DIVIDE    = 25;
defparam dcm_inst.CLKIN_PERIOD    = 10.000;
//synthesis translate_on

// Clock buffers
//------------------------
BUFG  buf_sys_clock  (.O(dco_clk), .I(dcm_clkfx));
*/


  clk_wiz_0 i_pll
 (
  .clk_in1 (CLK100MHZ),
  .reset (reset_pin),
  .locked (dcm_locked),
  .clk_out1 (dco_clk)
 );

//=============================================================================
// 4)  OPENMSP430 SYSTEM 0
//=============================================================================

omsp_system_0 omsp_system_0_inst (

// Clock & Reset
    .dco_clk           (dco_clk),                     // Fast oscillator (fast clock)
    .reset_n           (reset_n),                     // Reset Pin (low active, asynchronous and non-glitchy)

// Serial Debug Interface (I2C)
    .dbg_i2c_addr      (7'd50),                       // Debug interface: I2C Address
    .dbg_i2c_broadcast (7'd49),                       // Debug interface: I2C Broadcast Address (for multicore systems)
    .dbg_i2c_scl       (omsp_dbg_i2c_scl),            // Debug interface: I2C SCL
    .dbg_i2c_sda_in    (omsp_dbg_i2c_sda_in),         // Debug interface: I2C SDA IN
    .dbg_i2c_sda_out   (omsp0_dbg_i2c_sda_out),       // Debug interface: I2C SDA OUT

    .dbg_uart_txd      (dbg_uart_txd),
    .dbg_uart_rxd      (dbg_uart_rxd),

// Data Memory
    .dmem_addr         (omsp0_dmem_addr),             // Data Memory address
    .dmem_cen          (omsp0_dmem_cen),              // Data Memory chip enable (low active)
    .dmem_din          (omsp0_dmem_din),              // Data Memory data input
    .dmem_wen          (omsp0_dmem_wen),              // Data Memory write enable (low active)
    .dmem_dout         (omsp0_dmem_dout),             // Data Memory data output

// Program Memory
    .pmem_addr         (omsp0_pmem_addr),             // Program Memory address
    .pmem_cen          (omsp0_pmem_cen),              // Program Memory chip enable (low active)
    .pmem_din          (omsp0_pmem_din),              // Program Memory data input (optional)
    .pmem_wen          (omsp0_pmem_wen),              // Program Memory write enable (low active) (optional)
    .pmem_dout         (omsp0_pmem_dout),             // Program Memory data output

// UART
    .uart_rxd          (omsp0_uart_rxd),              // UART Data Receive (RXD)
    .uart_txd          (omsp0_uart_txd),              // UART Data Transmit (TXD)

// Switches & LEDs
    .switch            (sw),                 // Input switches
    .led               (led),                // LEDs
    .btn               (btn),
    .gpio_in           (gpio_in),
    .gpio_out          (gpio_out),
    .gpio_dir          (gpio_dir)
);



/*
//=============================================================================
// 5)  OPENMSP430 SYSTEM 1
//=============================================================================

omsp_system_1 omsp_system_1_inst (

// Clock & Reset
    .dco_clk           (dco_clk),                     // Fast oscillator (fast clock)
    .reset_n           (reset_n),                     // Reset Pin (low active, asynchronous and non-glitchy)

// Serial Debug Interface (I2C)
    .dbg_i2c_addr      (7'd51),                       // Debug interface: I2C Address
    .dbg_i2c_broadcast (7'd49),                       // Debug interface: I2C Broadcast Address (for multicore systems)
    .dbg_i2c_scl       (omsp_dbg_i2c_scl),            // Debug interface: I2C SCL
    .dbg_i2c_sda_in    (omsp_dbg_i2c_sda_in),         // Debug interface: I2C SDA IN
    .dbg_i2c_sda_out   (omsp1_dbg_i2c_sda_out),       // Debug interface: I2C SDA OUT

// Data Memory
    .dmem_addr         (omsp1_dmem_addr),             // Data Memory address
    .dmem_cen          (omsp1_dmem_cen),              // Data Memory chip enable (low active)
    .dmem_din          (omsp1_dmem_din),              // Data Memory data input
    .dmem_wen          (omsp1_dmem_wen),              // Data Memory write enable (low active)
    .dmem_dout         (omsp1_dmem_dout),             // Data Memory data output

// Program Memory
    .pmem_addr         (omsp1_pmem_addr),             // Program Memory address
    .pmem_cen          (omsp1_pmem_cen),              // Program Memory chip enable (low active)
    .pmem_din          (omsp1_pmem_din),              // Program Memory data input (optional)
    .pmem_wen          (omsp1_pmem_wen),              // Program Memory write enable (low active) (optional)
    .pmem_dout         (omsp1_pmem_dout),             // Program Memory data output

// Switches & LEDs
    .switch            (omsp_switch),                 // Input switches
    .led               (omsp1_led)                    // LEDs
);
*/

//=============================================================================
// 6)  PROGRAM AND DATA MEMORIES
//=============================================================================

/*
// Memory muxing (CPU 0)
assign omsp0_dmem_cen_sp =  omsp0_dmem_addr[`DMEM_MSB] | omsp0_dmem_cen;
assign omsp0_dmem_cen_dp = ~omsp0_dmem_addr[`DMEM_MSB] | omsp0_dmem_cen;
assign omsp0_dmem_dout   =  omsp0_dmem_dout_sel ? omsp0_dmem_dout_sp : omsp0_dmem_dout_dp;

always @ (posedge dco_clk or posedge dco_rst)
  if (dco_rst)                  omsp0_dmem_dout_sel <=  1'b1;
  else if (~omsp0_dmem_cen_sp)  omsp0_dmem_dout_sel <=  1'b1;
  else if (~omsp0_dmem_cen_dp)  omsp0_dmem_dout_sel <=  1'b0;

// Memory muxing (CPU 1)
assign omsp1_dmem_cen_sp =  omsp1_dmem_addr[`DMEM_MSB] | omsp1_dmem_cen;
assign omsp1_dmem_cen_dp = ~omsp1_dmem_addr[`DMEM_MSB] | omsp1_dmem_cen;
assign omsp1_dmem_dout   =  omsp1_dmem_dout_sel ? omsp1_dmem_dout_sp : omsp1_dmem_dout_dp;

always @ (posedge dco_clk or posedge dco_rst)
  if (dco_rst)                  omsp1_dmem_dout_sel <=  1'b1;
  else if (~omsp1_dmem_cen_sp)  omsp1_dmem_dout_sel <=  1'b1;
  else if (~omsp1_dmem_cen_dp)  omsp1_dmem_dout_sel <=  1'b0;

// Data Memory (CPU 0)
ram_16x1k_sp ram_16x1k_sp_dmem_omsp0 (
    .clka           ( dco_clk),
    .ena            (~omsp0_dmem_cen_sp),
    .wea            (~omsp0_dmem_wen),
    .addra          ( omsp0_dmem_addr[`DMEM_MSB-1:0]),
    .dina           ( omsp0_dmem_din),
    .douta          ( omsp0_dmem_dout_sp)
);

// Data Memory (CPU 1)
ram_16x1k_sp ram_16x1k_sp_dmem_omsp1 (
    .clka           ( dco_clk),
    .ena            (~omsp1_dmem_cen_sp),
    .wea            (~omsp1_dmem_wen),
    .addra          ( omsp1_dmem_addr[`DMEM_MSB-1:0]),
    .dina           ( omsp1_dmem_din),
    .douta          ( omsp1_dmem_dout_sp)
);

// Shared Data Memory
ram_16x1k_dp ram_16x1k_dp_dmem_shared (
    .clka           ( dco_clk),
    .ena            (~omsp0_dmem_cen_dp),
    .wea            (~omsp0_dmem_wen),
    .addra          ( omsp0_dmem_addr[`DMEM_MSB-1:0]),
    .dina           ( omsp0_dmem_din),
    .douta          ( omsp0_dmem_dout_dp),
    .clkb           ( dco_clk),
    .enb            (~omsp1_dmem_cen_dp),
    .web            (~omsp1_dmem_wen),
    .addrb          ( omsp1_dmem_addr[`DMEM_MSB-1:0]),
    .dinb           ( omsp1_dmem_din),
    .doutb          ( omsp1_dmem_dout_dp)
);

// Shared Program Memory
ram_16x8k_dp ram_16x8k_dp_pmem_shared (
    .clka           ( dco_clk),
    .ena            (~omsp0_pmem_cen),
    .wea            (~omsp0_pmem_wen),
    .addra          ( omsp0_pmem_addr),
    .dina           ( omsp0_pmem_din),
    .douta          ( omsp0_pmem_dout),
    .clkb           ( dco_clk),
    .enb            (~omsp1_pmem_cen),
    .web            (~omsp1_pmem_wen),
    .addrb          ( omsp1_pmem_addr),
    .dinb           ( omsp1_pmem_din),
    .doutb          ( omsp1_pmem_dout)
);

*/


  sp_mem #(
    .NB_COL(2),                           // Specify number of columns (number of bytes)
    .COL_WIDTH(8),                        // Specify column width (byte width, typically 8 or 9)
    .RAM_DEPTH(`PMEM_SIZE/2),                     // Specify RAM depth (number of entries)
    .RAM_PERFORMANCE("LOW_LATENCY"), // Select "HIGH_PERFORMANCE" or "LOW_LATENCY" 
    .INIT_FILE("")                        // Specify name/location of RAM initialization file if using one (leave blank if not)
  ) pmem (
    .addra(omsp0_pmem_addr),     // Address bus, width determined from RAM_DEPTH
    .dina(omsp0_pmem_din),       // RAM input data, width determined from NB_COL*COL_WIDTH
    .clka(dco_clk),       // Clock
    .wea(~omsp0_pmem_wen),         // Byte-write enable, width determined from NB_COL
    .ena(~omsp0_pmem_cen),         // RAM Enable, for additional power savings, disable port when not in use
    .rsta('0),       // Output reset (does not affect memory contents)
    .regcea('0),   // Output register enable
    .douta(omsp0_pmem_dout)      // RAM output data, width determined from NB_COL*COL_WIDTH
  );



  sp_mem #(
    .NB_COL(2),                           // Specify number of columns (number of bytes)
    .COL_WIDTH(8),                        // Specify column width (byte width, typically 8 or 9)
    .RAM_DEPTH(`DMEM_SIZE/2),                     // Specify RAM depth (number of entries)
    .RAM_PERFORMANCE("LOW_LATENCY"), // Select "HIGH_PERFORMANCE" or "LOW_LATENCY" 
    .INIT_FILE("")                        // Specify name/location of RAM initialization file if using one (leave blank if not)
  ) dmem (
    .addra(omsp0_dmem_addr),     // Address bus, width determined from RAM_DEPTH
    .dina(omsp0_dmem_din),       // RAM input data, width determined from NB_COL*COL_WIDTH
    .clka(dco_clk),       // Clock
    .wea(~omsp0_dmem_wen),         // Byte-write enable, width determined from NB_COL
    .ena(~omsp0_dmem_cen),         // RAM Enable, for additional power savings, disable port when not in use
    .rsta('0),       // Output reset (does not affect memory contents)
    .regcea('0),   // Output register enable
    .douta(omsp0_dmem_dout)      // RAM output data, width determined from NB_COL*COL_WIDTH
  );



//=============================================================================
// 7)  I/O CELLS
//=============================================================================

/*
//----------------------------------------------
// Micron N25Q128 SPI Flash
//   This is a Multi-I/O Flash.  Several pins
//  have dual purposes depending on the mode.
//----------------------------------------------
OBUF  SPI_CLK_PIN        (.I(1'b0),                  .O(SPI_SCK));
OBUF  SPI_CSN_PIN        (.I(1'b1),                  .O(SPI_CS_n));
IOBUF SPI_MOSI_MISO0_PIN (.T(1'b0), .I(1'b0), .O(),  .IO(SPI_MOSI_MISO0));
IOBUF SPI_MISO_MISO1_PIN (.T(1'b0), .I(1'b0), .O(),  .IO(SPI_MISO_MISO1));
OBUF  SPI_WN_PIN         (.I(1'b1),                  .O(SPI_Wn_MISO2));
OBUF  SPI_HOLD_PIN       (.I(1'b1),                  .O(SPI_HOLDn_MISO3));

//----------------------------------------------
// User DIP Switch x4
//----------------------------------------------
IBUF  SW3_PIN            (.O(omsp_switch[3]),        .I(GPIO_DIP4));
IBUF  SW2_PIN            (.O(omsp_switch[2]),        .I(GPIO_DIP3));
IBUF  SW1_PIN            (.O(omsp_switch[1]),        .I(GPIO_DIP2));
IBUF  SW0_PIN            (.O(omsp_switch[0]),        .I(GPIO_DIP1));

//----------------------------------------------
// User LEDs
//----------------------------------------------
OBUF  LED3_PIN           (.I(omsp1_led[1]),          .O(GPIO_LED4));
OBUF  LED2_PIN           (.I(omsp1_led[0]),          .O(GPIO_LED3));
OBUF  LED1_PIN           (.I(omsp0_led[1]),          .O(GPIO_LED2));
OBUF  LED0_PIN           (.I(omsp0_led[0]),          .O(GPIO_LED1));

//----------------------------------------------
// Silicon Labs CP2102 USB-to-UART Bridge Chip
//----------------------------------------------
IBUF  UART_RXD_PIN       (.O(omsp0_uart_rxd),        .I(USB_RS232_RXD));
OBUF  UART_TXD_PIN       (.I(omsp0_uart_txd),        .O(USB_RS232_TXD));

//----------------------------------------------
// Texas Instruments CDCE913 programming port
//----------------------------------------------
IOBUF SCL_PIN            (.T(1'b0), .I(1'b1), .O(),  .IO(SCL));
IOBUF SDA_PIN            (.T(1'b0), .I(1'b1), .O(),  .IO(SDA));

//----------------------------------------------
// Micron MT46H32M16LFBF-5 LPDDR
//----------------------------------------------

// Addresses
OBUF  LPDDR_A0_PIN       (.I(1'b0),                  .O(LPDDR_A0));
OBUF  LPDDR_A1_PIN       (.I(1'b0),                  .O(LPDDR_A1));
OBUF  LPDDR_A2_PIN       (.I(1'b0),                  .O(LPDDR_A2));
OBUF  LPDDR_A3_PIN       (.I(1'b0),                  .O(LPDDR_A3));
OBUF  LPDDR_A4_PIN       (.I(1'b0),                  .O(LPDDR_A4));
OBUF  LPDDR_A5_PIN       (.I(1'b0),                  .O(LPDDR_A5));
OBUF  LPDDR_A6_PIN       (.I(1'b0),                  .O(LPDDR_A6));
OBUF  LPDDR_A7_PIN       (.I(1'b0),                  .O(LPDDR_A7));
OBUF  LPDDR_A8_PIN       (.I(1'b0),                  .O(LPDDR_A8));
OBUF  LPDDR_A9_PIN       (.I(1'b0),                  .O(LPDDR_A9));
OBUF  LPDDR_A10_PIN      (.I(1'b0),                  .O(LPDDR_A10));
OBUF  LPDDR_A11_PIN      (.I(1'b0),                  .O(LPDDR_A11));
OBUF  LPDDR_A12_PIN      (.I(1'b0),                  .O(LPDDR_A12));
OBUF  LPDDR_BA0_PIN      (.I(1'b0),                  .O(LPDDR_BA0));
OBUF  LPDDR_BA1_PIN      (.I(1'b0),                  .O(LPDDR_BA1));

// Data
IOBUF LPDDR_DQ0_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ0));
IOBUF LPDDR_DQ1_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ1));
IOBUF LPDDR_DQ2_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ2));
IOBUF LPDDR_DQ3_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ3));
IOBUF LPDDR_DQ4_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ4));
IOBUF LPDDR_DQ5_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ5));
IOBUF LPDDR_DQ6_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ6));
IOBUF LPDDR_DQ7_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ7));
IOBUF LPDDR_DQ8_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ8));
IOBUF LPDDR_DQ9_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ9));
IOBUF LPDDR_DQ10_PIN     (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ10));
IOBUF LPDDR_DQ11_PIN     (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ11));
IOBUF LPDDR_DQ12_PIN     (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ12));
IOBUF LPDDR_DQ13_PIN     (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ13));
IOBUF LPDDR_DQ14_PIN     (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ14));
IOBUF LPDDR_DQ15_PIN     (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_DQ15));
OBUF  LPDDR_LDM_PIN      (.I(1'b0),                  .O(LPDDR_LDM));
OBUF  LPDDR_UDM_PIN      (.I(1'b0),                  .O(LPDDR_UDM));
IOBUF LPDDR_LDQS_PIN     (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_LDQS));
IOBUF LPDDR_UDQS_PIN     (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_UDQS));

// Clock
IOBUF LPDDR_CK_N_PIN     (.T(1'b1), .I(1'b0), .O(),  .IO(LPDDR_CK_N));
IOBUF LPDDR_CK_P_PIN     (.T(1'b1), .I(1'b1), .O(),  .IO(LPDDR_CK_P));
OBUF  LPDDR_CKE_PIN      (.I(1'b0),                  .O(LPDDR_CKE));

// Control
OBUF  LPDDR_CAS_N_PIN    (.I(1'b1),                  .O(LPDDR_CAS_n));
OBUF  LPDDR_RAS_N_PIN    (.I(1'b1),                  .O(LPDDR_RAS_n));
OBUF  LPDDR_WE_N_PIN     (.I(1'b1),                  .O(LPDDR_WE_n));
IOBUF LPDDR_RZQ_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(LPDDR_RZQ));


//----------------------------------------------
// National Semiconductor DP83848J 10/100 Ethernet PHY
//   Pull-ups on RXD are necessary to set the PHY AD to 11110b.
//   Must keep the PHY from defaulting to PHY AD = 00000b
//   because this is Isolate Mode
//----------------------------------------------
IBUF  ETH_COL_PIN        (.O(),                      .I(ETH_COL));
IBUF  ETH_CRS_PIN        (.O(),                      .I(ETH_CRS));
OBUF  ETH_MDC_PIN        (.I(1'b0),                  .O(ETH_MDC));
IOBUF ETH_MDIO_PIN       (.T(1'b0), .I(1'b0), .O(),  .IO(ETH_MDIO));
OBUF  ETH_RESET_N_PIN    (.I(1'b1),                  .O(ETH_RESET_n));
IBUF  ETH_RX_CLK_PIN     (.O(),                      .I(ETH_RX_CLK));
IBUF  ETH_RX_D0_PIN      (.O(),                      .I(ETH_RX_D0));
IBUF  ETH_RX_D1_PIN      (.O(),                      .I(ETH_RX_D1));
IBUF  ETH_RX_D2_PIN      (.O(),                      .I(ETH_RX_D2));
IBUF  ETH_RX_D3_PIN      (.O(),                      .I(ETH_RX_D3));
IBUF  ETH_RX_DV_PIN      (.O(),                      .I(ETH_RX_DV));
IBUF  ETH_RX_ER_PIN      (.O(),                      .I(ETH_RX_ER));
IBUF  ETH_TX_CLK_PIN     (.O(),                      .I(ETH_TX_CLK));
OBUF  ETH_TX_D0_PIN      (.I(1'b0),                  .O(ETH_TX_D0));
OBUF  ETH_TX_D1_PIN      (.I(1'b0),                  .O(ETH_TX_D1));
OBUF  ETH_TX_D2_PIN      (.I(1'b0),                  .O(ETH_TX_D2));
OBUF  ETH_TX_D3_PIN      (.I(1'b0),                  .O(ETH_TX_D3));
OBUF  ETH_TX_EN_PIN      (.I(1'b0),                  .O(ETH_TX_EN));

//----------------------------------------------
// Peripheral Modules (PMODs) and GPIO
//     https://www.digilentinc.com/PMODs
//----------------------------------------------

assign omsp_dbg_i2c_sda_out = omsp0_dbg_i2c_sda_out & omsp1_dbg_i2c_sda_out;

// Connector J5
IOBUF PMOD1_P1_PIN       (.T(1'b0),                  .I(1'b0), .O(),                     .IO(PMOD1_P1));
IOBUF PMOD1_P2_PIN       (.T(1'b0),                  .I(1'b0), .O(),                     .IO(PMOD1_P2));
IOBUF PMOD1_P3_PIN       (.T(omsp_dbg_i2c_sda_out),  .I(1'b0), .O(omsp_dbg_i2c_sda_in),  .IO(PMOD1_P3));
IBUF  PMOD1_P4_PIN       (                                     .O(omsp_dbg_i2c_scl),     .I (PMOD1_P4));
IOBUF PMOD1_P7_PIN       (.T(1'b0),                  .I(1'b0), .O(),                     .IO(PMOD1_P7));
IBUF  PMOD1_P8_PIN       (                                     .O(),                     .I (PMOD1_P8));
IOBUF PMOD1_P9_PIN       (.T(1'b0),                  .I(1'b0), .O(),                     .IO(PMOD1_P9));
IOBUF PMOD1_P10_PIN      (.T(1'b0),                  .I(1'b0), .O(),                     .IO(PMOD1_P10));

// Connector J4
IOBUF PMOD2_P1_PIN       (.T(1'b0), .I(1'b0), .O(),  .IO(PMOD2_P1));
IOBUF PMOD2_P2_PIN       (.T(1'b0), .I(1'b0), .O(),  .IO(PMOD2_P2));
IOBUF PMOD2_P3_PIN       (.T(1'b0), .I(1'b0), .O(),  .IO(PMOD2_P3));
IOBUF PMOD2_P4_PIN       (.T(1'b0), .I(1'b0), .O(),  .IO(PMOD2_P4));
IOBUF PMOD2_P7_PIN       (.T(1'b0), .I(1'b0), .O(),  .IO(PMOD2_P7));
IOBUF PMOD2_P8_PIN       (.T(1'b0), .I(1'b0), .O(),  .IO(PMOD2_P8));
IOBUF PMOD2_P9_PIN       (.T(1'b0), .I(1'b0), .O(),  .IO(PMOD2_P9));
IOBUF PMOD2_P10_PIN      (.T(1'b0), .I(1'b0), .O(),  .IO(PMOD2_P10));

*/

//=============================================================================
//8)  CHIPSCOPE
//=============================================================================


/*
//`define WITH_CHIPSCOPE
`ifdef WITH_CHIPSCOPE

// Sampling clock
reg [7:0] div_cnt;
always @ (posedge dco_clk or posedge dco_rst)
  if (dco_rst)           div_cnt <=  8'h00;
  else if (div_cnt > 10) div_cnt <=  8'h00;
  else                   div_cnt <=  div_cnt+8'h01;

reg clk_sample;
always @ (posedge dco_clk or posedge dco_rst)
  if (dco_rst) clk_sample <=  1'b0;
  else         clk_sample <=  (div_cnt==8'h00);


// ChipScope instance
wire        [35:0] chipscope_control;
chipscope_ila chipscope_ila (
    .CONTROL  (chipscope_control),
    .CLK      (clk_sample),
    .TRIG0    (chipscope_trigger)
);

chipscope_icon chipscope_icon (
    .CONTROL0 (chipscope_control)
);


assign chipscope_trigger[0]     = 1'b0;
assign chipscope_trigger[1]     = 1'b0;
assign chipscope_trigger[2]     = 1'b0;
assign chipscope_trigger[23:3]  = 21'h00_0000;
`endif
*/

  assign dbg_uart_rxd = uart_txd_in;
  assign uart_rxd_out = dbg_uart_txd;

  assign gpio_in[0] = ck_io0;
  assign gpio_in[1] = ck_io1;
  assign gpio_in[2] = ck_io2;
  assign gpio_in[3] = ck_io3;
  assign gpio_in[4] = ck_io4;
  assign gpio_in[5] = ck_io5;
  assign gpio_in[6] = ck_io6;
  assign gpio_in[7] = ck_io7;

  assign ck_io0 = gpio_dir[0] ? gpio_out[0] : 1'bz;
  assign ck_io1 = gpio_dir[1] ? gpio_out[1] : 1'bz;
  assign ck_io2 = gpio_dir[2] ? gpio_out[2] : 1'bz;
  assign ck_io3 = gpio_dir[3] ? gpio_out[3] : 1'bz;
  assign ck_io4 = gpio_dir[4] ? gpio_out[4] : 1'bz;
  assign ck_io5 = gpio_dir[5] ? gpio_out[5] : 1'bz;
  assign ck_io6 = gpio_dir[6] ? gpio_out[6] : 1'bz;
  assign ck_io7 = gpio_dir[7] ? gpio_out[7] : 1'bz;


  assign omsp0_uart_rxd = ck_io8;
  assign ck_io9 = omsp0_uart_txd;

endmodule
