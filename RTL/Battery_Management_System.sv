//=============================================================================
//
//Module Name:					Battery_Management_System.sv
//Department:					Xidian University
//Function Description:	        MAX17263电池管理系统驱动模块
//
//------------------------------------------------------------------------------
//
//Version 	Design		Coding		Simulata	  Review		Rel data
//V1.0		Verdvana	Verdvana	Verdvana		  			2020-10-08
//
//------------------------------------------------------------------------------
//
//Version	Modified History
//V1.0		Slave address：0x6C（8bit）or 0x36（7：bit）
//
//=============================================================================

`timescale 1ns/1ps

module Battery_Management_System (
    input  logic        clk_100m,       //系统时钟
    input  logic        rst_n,          //异步复位

    input  logic [3:0]  button,         //按键

    output logic        scl,
    inout  wire         sda

);


    logic [7:0]     device_addr;
    logic [7:0]     word_addr;
    logic           measure_done;

    logic           num_word_addr;
    logic [7:0]     num_data_w;
    logic [7:0]     num_data_r;

    logic           wen;
    logic [7:0]     wdata;
    logic           wvalid;

    logic           ren;
    logic [7:0]     rdata;
    logic           rvalid;

    logic           ready;
    logic           done;
    logic           error;

    logic           en;

    logic [15:0]    fullcap;
    logic [15:0]    repcap;
    logic [15:0]    repsoc;
    logic [15:0]    age;
    logic [15:0]    thrmtemp;
    
    logic           sda_i,sda_o,sda_en;

    assign  sda      = sda_en ? sda_o : 1'bz;
    assign  sda_i    = sda;


    Button #(
        .BUTTON_WIDTH(1                 ),   //按键位宽
        .SYS_CLOCK   (100_000_000       ),   //时钟频率
        .FILTER_DELAY(5                 )    //消抖延迟（单位：ms）
    )u_Button(
        .clk         (clk_100m          ),   //时钟
        .rst_n       (rst_n             ),   //异步复位

        .button_in   (~button[0]        ),   //按键输入

        .button_out  (                  ),   //消抖后输出
        .button_edge (read              )    //消抖后输出一个周期
    );


    I2C_Controller #(
        .SYS_CLOCK      (100_000_000    ),
        .SCL_CLOCK      (400_000        )
    )u_I2C_Controller(
        .clk            (clk_100m       ),
        .rst_n          (rst_n          ),

        .device_addr    (device_addr    ),
        .word_addr_h    (               ),
        .word_addr_l    (word_addr      ),

        .num_word_addr  (num_word_addr  ),
        .num_data_w     (num_data_w     ),
        .num_data_r     (num_data_r     ),

        .wen            (wen            ),
        .wdata          (wdata          ),
        .wvalid         (wvalid         ),

        .ren            (ren            ),
        .rdata          (rdata          ),
        .rvalid         (rvalid         ),

        .scl            (scl            ),
        .sda_i          (sda_i          ),
        .sda_o          (sda_o          ),
        .sda_en         (sda_en         ),

        .ready          (ready          ),
        .error          (error          ),
        .done           (done           )
    );

    MAX17263_Driver #(
        .SYS_CLOCK      (100_000_000    )   //时钟频率
    )u_MAX17263_Driver(
    //System Interface
    .clk                (clk_100m       ),
    .rst_n                               ,
    //Master Interface
    .measure_start      (read           ),
    .nominal_cap        (16'h0d16       ),
    .measure_cycle      (1'b0           ),  //循环测量模式，1为循环测量，0单次测量
    .measure_done                        ,  //测量完成
    //I2C Control Interface
    .device_addr                         ,  //器件地址（8bit格式，最后1bit会被截掉）
    .word_addr                           ,  //字节地址

    .num_word_addr                       ,  //字节地址宽度，0为八位，1为十六位
    .num_data_w                          ,  //写数据(8bit)的个数（num_data_w+1）
    .num_data_r                          ,  //读数据(8bit)的个数（num_data_r+1）    

    .wen                                 ,  //写使能
    .wdata                               ,  //写数据
    .wvalid                              ,  //写数据有效（写完一个8bit数据会保持一个周期上升沿）

    .ren                                 ,  //读使能
    .rdata                               ,  //读数据
    .rvalid                              ,  //读数据有效（读完一个8bit数据会保持一个周期上升沿）

    .ready                               ,  //i2c ready信号
    .done                                ,  //i2c 传输完成
    .error                               ,  //i2c 传输错误
    //Read Data
    .fullcap                             ,
    .repcap                              ,
    .repsoc                              ,
    .age                                 ,
    .thrmtemp
);


endmodule