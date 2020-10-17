//=============================================================================
//
//Module Name:					MAX17263_Driver.sv
//Department:					Xidian University
//Function Description:	        MAX17263驱动器
//
//------------------------------------------------------------------------------
//
//Version 	Design		Coding		Simulata	  Review		Rel data
//V1.0		Verdvana	Verdvana	Verdvana		  			2020-10-09
//
//------------------------------------------------------------------------------
//
//Version	Modified History
//V1.0		驱动MAX17263初始化，然后读取电池设计容量，总容量，剩余容量等；
//          循环测量和单次测量两种模式；
//          读取的mha值除以2则为最终结果；
//          读取的百分比值除以256则为最终结果；
//          读取的温度值除以256则为最终结果；
//          Slave address：0x6C（8bit）or 0x36（7：bit）。
//              
//
//=============================================================================

`timescale 1ns/1ps

module MAX17263_Driver#(
    parameter           SYS_CLOCK = 100_000_000
)(
    //System Interface
    input  logic        clk,
    input  logic        rst_n,
    //Master Interface
    input  logic        measure_start,  //工作使能
    input  logic [15:0] nominal_cap,    //标称容量（mha）
    input  logic        measure_cycle,  //循环测量模式，1为循环测量，0单次测量
    output logic        measure_done,   //测量完成，测量数据可读
    //I2C Control Interface
    output logic [7:0]  device_addr,    //器件地址（8bit格式，最后1bit会被截掉）
    output logic [7:0]  word_addr,      //字节地址

    output logic        num_word_addr,  //字节地址宽度，0为八位，1为十六位
    output logic [7:0]  num_data_w,     //写数据(8bit)的个数（num_data_w+1）
    output logic [7:0]  num_data_r,     //读数据(8bit)的个数（num_data_r+1）    

    output logic        wen,            //写使能
    output logic [7:0]  wdata,          //写数据
    input  logic        wvalid,         //写数据有效（写完一个8bit数据会保持一个周期上升沿）

    output logic        ren,            //读使能
    input  logic [7:0]  rdata,          //读数据
    input  logic        rvalid,         //读数据有效（读完一个8bit数据会保持一个周期上升沿）

    input  logic        ready,          //i2c ready信号
    input  logic        done,           //i2c 传输完成
    input  logic        error,          //i2c 传输错误
    //Read Data
    output logic [15:0] fullcap,        //电池容量
    output logic [15:0] repcap,         //电池剩余容量
    output logic [15:0] repsoc,         //电池剩余容量百分比
    output logic [15:0] age,            //电池健康度百分比
    output logic [15:0] thrmtemp        //热敏电阻温度
);


    //======================================================================
    //参数定义
    parameter       TCO             = 1,
                    DELAY_CNT       = SYS_CLOCK/1_000_000-1,
                    //设备地址
                    DEVICE_ADDR     = 8'h6C,
                    //寄存器地址
                    STATUS_ADDR     = 8'h00,
                    FSTAT_ADDR      = 8'h3d,
                    SOFTWAKEUP_ADDR = 8'h60,
                    HIBCFG_ADDR     = 8'hba,
                    CONFIG_ADDR     = 8'h1d,
                    DESIGNCAP_ADDR  = 8'h18,
                    MODELCFG_ADDR   = 8'hdb,
                    REPSOC_ADDR     = 8'h06,
                    REPCAP_ADDR     = 8'h05,
                    ATTTE_ADDR      = 8'hdd,
                    ATAVCAP_ADDR    = 8'hdf,
                    FULLCAPREP_ADDR = 8'h10,
                    TEMP_ADDR       = 8'h08,
                    AGE_ADDR        = 8'h07;


    //======================================================================
    //位宽计算函数
    function integer clogb2 (input integer depth);
    begin
        for (clogb2=0; depth>0; clogb2=clogb2+1) 
            depth = depth >>1;                          
    end
    endfunction

    //======================================================================
    //信号定义

    logic [15:0]    data_out;       //16bit数据输出
    logic [clogb2(DELAY_CNT)-1:0]   delay_cnt;  //延时计数器

    logic [15:0]    hibcfg_orig;    //hibcfg原始数据
    logic [15:0]    status_orig;    //status原始数据
    logic [15:0]    config_orig;    //config

    logic           wdata_cnt;      //两字节计数器

    logic [15:0]    fullcap_r;      //电池容量
    logic [15:0]    repcap_r;       //电池剩余容量
    logic [15:0]    repsoc_r;       //电池剩余容量百分比
    logic [15:0]    age_r;          //电池健康度
    logic [15:0]    thrmtemp_r;     //热敏电阻温度

    //======================================================================
    //状态定义
    enum logic[4:0] {
        IDLE        ,
        DELAY       ,   //延时1ms
        STATUS_S    ,   //起始读STATUS
        STATUS_W    ,   //写STATUS
        STATUS_R    ,   //读STATUS
        STATUS_V    ,   //读STATUS验证
        FSTAT       ,   //读FSTAT
        CLEAR       ,   //写SOFTWAKEUP全零
        HIBCFG_R    ,   //读HIBCFG
        HIBCFG_CL   ,   //写HIBCFG全零
        HIBCFG_W    ,   //写HIBCFG
        SOFTWAKEUP  ,   //写SOFTWAKEUP
        CONFIG_R    ,   //读CONFIG
        CONFIG_W    ,   //写CONFIG
        DESIGNCAP   ,   //写DESIGNCAP
        MODELCFG_W  ,   //写MODELCFG
        MODELCFG_R  ,   //读MODELCFG
        FULLCAPREP  ,   //读FULLCAPREP
        REPSOC      ,   //读REPSOC
        REPCAP      ,   //读REPCAP
        AGE         ,   //读电池健康度
        TEMP        ,   //读TEMP
        MEASUREDONE     //测量完成
    }state,next_state;


    //======================================================================
    //I2C 配置
    assign          device_addr     = DEVICE_ADDR;
    assign          num_word_addr   = 1'b0;
    assign          num_data_w      = 1'b1;
    assign          num_data_r      = 1'b1;


    //======================================================================
    //状态机

    //状态跳转
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            state       <= #TCO IDLE;
        else
            state       <= #TCO next_state;
    end

    //状态解码
    always_comb begin
        case(state)
            IDLE:
                if(measure_start)               //测量开始
                    next_state  = STATUS_S;
                else
                    next_state  = IDLE;
            //Initialize Registers to Recommended Configuration
            STATUS_S:
                if(done)
                    if(data_out[1])             //第1位为1则开始配置
                        next_state  = FSTAT;
                    else                        //否则为配置完成，可以直接测量
                        next_state  = STATUS_V;
                else
                    next_state  = STATUS_S;
            FSTAT:
                if(done && (data_out[0]==1'b0)) //第0位为0
                    next_state  = HIBCFG_R;
                else
                    next_state  = FSTAT;
            //Initialize Configuration
            HIBCFG_R:
                if(done)
                    next_state  = SOFTWAKEUP;
                else
                    next_state  = HIBCFG_R;
            SOFTWAKEUP:
                if(done)
                    next_state  = CLEAR;
                else
                    next_state  = SOFTWAKEUP;
            CLEAR:
                if(done)
                    next_state  = CONFIG_R;
                else
                    next_state  = CLEAR;
            CONFIG_R:
                if(done)
                    next_state  = DESIGNCAP;
                else
                    next_state  = CONFIG_R;
            DESIGNCAP:
                if(done)
                    next_state  = CONFIG_W;
                else
                    next_state  = DESIGNCAP;
            CONFIG_W:
                if(done)
                    next_state  = MODELCFG_W;
                else
                    next_state  = CONFIG_W;            
            MODELCFG_W:
                if(done)
                    next_state  = MODELCFG_R;
                else
                    next_state  = MODELCFG_W;
            MODELCFG_R:
                if(done&&(~data_out[15]))       //最高位为0
                    next_state  = HIBCFG_W;
                else
                    next_state  = MODELCFG_R;
            HIBCFG_W:     
                if(done)
                    next_state  = STATUS_W;
                else
                    next_state  = HIBCFG_W;   
            STATUS_R:
                if(done)
                    next_state  = STATUS_W;
                else
                    next_state  = STATUS_R; 
            STATUS_W:
                if(done)
                    next_state  = DELAY;
                else
                    next_state  = STATUS_W;
            DELAY:
                if(&delay_cnt)
                    next_state  = STATUS_V;
                else
                    next_state  = DELAY;
            STATUS_V:
                if(done)
                    if(data_out[1])             //是否配置完成
                        next_state  = STATUS_S;
                    else
                        next_state  = FULLCAPREP;
                else
                    next_state  = STATUS_V;  
            //Monitor the Battery
            FULLCAPREP:
                if(done)
                    next_state  = REPSOC;
                else
                    next_state  = FULLCAPREP;
            REPSOC:
                if(done)
                    next_state  = REPCAP;
                else
                    next_state  = REPSOC;
            REPCAP:
                if(done)
                    next_state  = AGE;
                else
                    next_state  = REPCAP;
            AGE:
                if(done)
                    next_state  = TEMP;
                else
                    next_state  = AGE;
            TEMP:
                if(done)
                    next_state  = MEASUREDONE;
                else
                    next_state  = TEMP;
            MEASUREDONE:
                if(measure_cycle)
                    next_state  = FULLCAPREP;
                else
                    next_state  = IDLE;
            default:
                next_state  = IDLE;
        endcase
    end

    //======================================================================
    //延时
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            delay_cnt   <= #TCO '0;
        else
            case(state)
                DELAY:
                    delay_cnt   <= #TCO delay_cnt + 1;
                default:
                    delay_cnt   <= #TCO '0;
            endcase
    end

    //======================================================================
    //原始数据寄存
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            hibcfg_orig <= #TCO '0;
        else
            case(state)
                HIBCFG_R:
                    if(done)
                        hibcfg_orig <= #TCO data_out;
                    else
                        hibcfg_orig <= #TCO hibcfg_orig;
                default:
                    hibcfg_orig <= #TCO hibcfg_orig;
            endcase
    end

    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            status_orig <= #TCO '0;
        else
            case(state)
                STATUS_R:
                    if(done)
                        status_orig <= #TCO data_out;
                    else
                        status_orig <= #TCO status_orig;
                default:
                    status_orig <= #TCO status_orig;
            endcase
    end

    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            config_orig <= #TCO '0;
        else
            case(state)
                CONFIG_R:
                    if(done)
                        config_orig <= #TCO data_out;
                    else
                        config_orig <= #TCO config_orig;
                default:
                    config_orig <= #TCO config_orig;
            endcase
    end

    //======================================================================
    //寄存器地址
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            word_addr   <= #TCO '0;
        else
            case(state)
                STATUS_S,STATUS_W,STATUS_R,STATUS_V:     
                    word_addr   <= #TCO STATUS_ADDR;
                FSTAT:      
                    word_addr   <= #TCO FSTAT_ADDR;
                CLEAR,SOFTWAKEUP:
                    word_addr   <= #TCO SOFTWAKEUP_ADDR;
                HIBCFG_R,HIBCFG_CL,HIBCFG_W:
                    word_addr   <= #TCO HIBCFG_ADDR;
                CONFIG_W,CONFIG_R:
                    word_addr   <= #TCO CONFIG_ADDR;
                DESIGNCAP:  
                    word_addr   <= #TCO DESIGNCAP_ADDR;
                MODELCFG_W,MODELCFG_R: 
                    word_addr   <= #TCO MODELCFG_ADDR;
                FULLCAPREP: 
                    word_addr   <= #TCO FULLCAPREP_ADDR;
                REPSOC:     
                    word_addr   <= #TCO REPSOC_ADDR;
                REPCAP:     
                    word_addr   <= #TCO REPCAP_ADDR;
                AGE:
                    word_addr   <= #TCO AGE_ADDR;
                TEMP:
                    word_addr   <= #TCO TEMP_ADDR;
                default:    
                    word_addr   <= #TCO '0;
            endcase
    end

    //======================================================================
    //写 

    //写使能
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            wen <= #TCO '0;
        else 
            case(state)
                CLEAR,SOFTWAKEUP,STATUS_W,HIBCFG_CL,HIBCFG_W,DESIGNCAP,CONFIG_W,MODELCFG_W:
                    if(ready)
                        wen <= #TCO '1;
                    else if(done)
                        wen <= #TCO '0;
                    else
                        wen <= #TCO wen;
                default:
                    wen <= #TCO wen;
            endcase
    end

    //写数据  
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            wdata_cnt   <= #TCO '0;
        else
            case(state)
                CLEAR,SOFTWAKEUP,STATUS_W,HIBCFG_CL,HIBCFG_W,DESIGNCAP,CONFIG_W,MODELCFG_W:
                    if(wvalid)
                        wdata_cnt   <= #TCO wdata_cnt + 1;
                    else
                        wdata_cnt   <= #TCO wdata_cnt;
                default:
                    wdata_cnt   <= #TCO '0;
            endcase
    end

    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            wdata   <= #TCO '0;
        else
            case(state)
                SOFTWAKEUP:
                    case(wdata_cnt)
                        '0:         wdata   <= #TCO 8'h00;
                        '1:         wdata   <= #TCO 8'h90;
                        default:    wdata   <= #TCO 8'h00;
                    endcase
                HIBCFG_CL:
                    case(wdata_cnt)
                        '0:         wdata   <= #TCO 8'h00;
                        '1:         wdata   <= #TCO 8'h00;
                        default:    wdata   <= #TCO 8'h00;
                    endcase
                CLEAR:
                    case(wdata_cnt)
                        '0:         wdata   <= #TCO 8'h00;
                        '1:         wdata   <= #TCO 8'h00;
                        default:    wdata   <= #TCO 8'h00;
                    endcase
                DESIGNCAP:
                    case(wdata_cnt)
                        '0:         wdata   <= #TCO {nominal_cap[6:0],1'b0};    //写入的是电池容量的两倍值
                        '1:         wdata   <= #TCO nominal_cap[14:7];
                        default:    wdata   <= #TCO 8'h00;
                    endcase
                CONFIG_W:
                    case(wdata_cnt)
                        '0:         wdata   <= #TCO config_orig[7:0];
                        '1:         wdata   <= #TCO {1'b1,config_orig[14:8]};
                        default:    wdata   <= #TCO 8'h00;
                    endcase
                MODELCFG_W:
                    case(wdata_cnt)
                        '0:         wdata   <= #TCO 8'h00;
                        '1:         wdata   <= #TCO 8'h84;
                        default:    wdata   <= #TCO 8'h00;
                    endcase
                HIBCFG_W:
                    case(wdata_cnt)
                        '0:         wdata   <= #TCO hibcfg_orig[7:0];
                        '1:         wdata   <= #TCO hibcfg_orig[15:8];
                        default:    wdata   <= #TCO 8'h00;
                    endcase
                STATUS_W:
                    case(wdata_cnt)
                        '0:         wdata   <= #TCO {status_orig[7:2],1'b0,status_orig[0]};
                        '1:         wdata   <= #TCO status_orig[15:8];
                        default:    wdata   <= #TCO 8'h00;
                    endcase
                default:
                    wdata   <= #TCO '0;
            endcase
    end

    //======================================================================
    //读

    //读使能
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            ren <= #TCO '0;
        else 
            case(state)
                STATUS_S,STATUS_R,STATUS_V,FSTAT,HIBCFG_R,CONFIG_R,MODELCFG_R,FULLCAPREP,REPSOC,REPCAP,AGE,TEMP:
                    if(ready)
                        ren <= #TCO '1;
                    else if(rvalid)
                        ren <= #TCO '0;
                    else
                        ren <= #TCO ren;
                default:
                    ren <= #TCO '0;
            endcase
    end

    //读数据
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            data_out    <= #TCO '0;
        else if(rvalid)
            data_out    <= #TCO {rdata,data_out[15:8]};
        else
            data_out    <= #TCO data_out;
    end

    //======================================================================
    //读数据输出
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)begin
            fullcap_r <= #TCO '0;
            repcap_r  <= #TCO '0;
            repsoc_r  <= #TCO '0;
            age_r     <= #TCO '0;
            thrmtemp_r<= #TCO '0;
        end
        else if(done)
            case(state)
                FULLCAPREP:begin
                    fullcap_r <= #TCO data_out;
                    repcap_r  <= #TCO repcap_r;
                    repsoc_r  <= #TCO repsoc_r;
                    age_r     <= #TCO age_r;
                    thrmtemp_r<= #TCO thrmtemp_r;
                end
                REPCAP:begin
                    fullcap_r <= #TCO fullcap_r;
                    repcap_r  <= #TCO data_out;
                    repsoc_r  <= #TCO repsoc_r;
                    age_r     <= #TCO age_r;
                    thrmtemp_r<= #TCO thrmtemp_r;
                end
                REPSOC:begin
                    fullcap_r <= #TCO fullcap_r;
                    repcap_r  <= #TCO repcap_r;
                    repsoc_r  <= #TCO data_out;
                    age_r     <= #TCO age_r;
                    thrmtemp_r<= #TCO thrmtemp_r;
                end
                AGE:begin
                    fullcap_r <= #TCO fullcap_r;
                    repcap_r  <= #TCO repcap_r;
                    repsoc_r  <= #TCO repsoc_r;
                    age_r     <= #TCO data_out;
                    thrmtemp_r<= #TCO thrmtemp_r;
                end
                TEMP:begin
                    fullcap_r <= #TCO fullcap_r;
                    repcap_r  <= #TCO repcap_r;
                    repsoc_r  <= #TCO repsoc_r;
                    age_r     <= #TCO age_r;
                    thrmtemp_r<= #TCO data_out;
                end
                default:begin
                    fullcap_r <= #TCO fullcap_r;
                    repcap_r  <= #TCO repcap_r;
                    repsoc_r  <= #TCO repsoc_r;
                    age_r     <= #TCO age_r;
                    thrmtemp_r<= #TCO thrmtemp_r;
                end
            endcase
        else begin
            fullcap_r <= #TCO fullcap_r;
            repcap_r  <= #TCO repcap_r;
            repsoc_r  <= #TCO repsoc_r;
            age_r     <= #TCO age_r;
            thrmtemp_r<= #TCO thrmtemp_r;
        end
    end

    //======================================================================
    //测量完成标志输出
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            measure_done    <= #TCO '0;
        else
            case(state)
                MEASUREDONE:
                    measure_done    <= #TCO '1;
                default:
                    measure_done    <= #TCO '0;
            endcase
    end

    //======================================================================
    //测量数据输出
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)begin
            fullcap <= #TCO '0;
            repcap  <= #TCO '0;
            repsoc  <= #TCO '0;
            age     <= #TCO '0;
            thrmtemp<= #TCO '0;
        end
        else begin
            case(state)    
                MEASUREDONE:begin
                    fullcap <= #TCO fullcap_r;
                    repcap  <= #TCO repcap_r;
                    repsoc  <= #TCO repsoc_r;
                    age     <= #TCO age_r;
                    thrmtemp<= #TCO thrmtemp_r;
                end
                default:begin
                    fullcap <= #TCO fullcap;
                    repcap  <= #TCO repcap;
                    repsoc  <= #TCO repsoc;
                    age     <= #TCO age;
                    thrmtemp<= #TCO thrmtemp;
                end
            endcase
        end
    end


endmodule