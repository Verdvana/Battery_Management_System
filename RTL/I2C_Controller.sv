//=============================================================================
//
//Module Name:					I2C_Controller.sv
//Department:					Xidian University
//Function Description:	        I2C控制器
//
//------------------------------------------------------------------------------
//
//Version 	Design		Coding		Simulata	  Review		Rel data
//V1.0		Verdvana	Verdvana	Verdvana		  			2020-09-26
//V1.1		Verdvana	Verdvana	Verdvana		  			2020-10-07
//V1.2		Verdvana	Verdvana	Verdvana		  			2020-10-07
//
//------------------------------------------------------------------------------
//
//Version	Modified History
//V1.0		单字节读写
//          系统时钟和I2C时钟可定制	
//V1.1      两字节地址单字节数据读写
//V1.2      两字节地址多字节数据读写
//V1.3      在每次数据传输之前加入两个SCL周期的延时
//          保证上一次传输结束与本次传输开始之间不冲突
//
//=============================================================================

`timescale 1ns/1ps

module I2C_Controller #(
    parameter   SYS_CLOCK = 50_000_000, //系统时钟频率
                SCL_CLOCK = 400_000     //I2C时钟频率
)(
    input  logic        clk,            //系统时钟
    input  logic        rst_n,          //异步复位

    input  logic [7:0]  device_addr,    //器件地址（8bit格式，最后1bit会被截掉）
    input  logic [7:0]  word_addr_h,    //字节地址高八位
    input  logic [7:0]  word_addr_l,    //字节地址低八位

    input  logic        num_word_addr,  //字节地址宽度，0为八位，1为十六位
    input  logic [7:0]  num_data_w,     //写数据(8bit)的个数（num_data_w+1）
    input  logic [7:0]  num_data_r,     //读数据(8bit)的个数（num_data_r+1）

    input  logic        wen,            //写使能
    input  logic [7:0]  wdata,          //写数据
    output logic        wvalid,         //写数据有效（写完一个8bit数据会保持一个周期上升沿）

    input  logic        ren,            //读使能
    output logic [7:0]  rdata,          //读数据
    output logic        rvalid,         //读数据有效（读完一个8bit数据会保持一个周期上升沿）

    output logic        scl,            //i2c scl输出
    input  logic        sda_i,          //i2c sda输入
    output logic        sda_o,          //i2c sda输出
    output logic        sda_en,         //i2c sda输出有效

    output logic        ready,          //准备好传输
    output logic        done,           //i2c 传输完成
    output logic        error           //i2c 传输错误
);

    //位宽计算函数
    function integer clogb2 (input integer depth);
    begin
        for (clogb2=0; depth>0; clogb2=clogb2+1) 
            depth = depth >>1;                          
    end
    endfunction

    localparam  SCL_CLK_CNT = SYS_CLOCK/SCL_CLOCK,  //scl时钟计数值
                TCO         = 1;                    //寄存器延迟

    logic           wr;                 //读写判断：写为1，读为0
    logic           wen_r;              //写使能寄存
    logic           ren_r;              //读使能寄存

    logic [15:0]    delay_cnt;          //延时计数器

    logic [7:0]     device_addr_r;      //器件地址寄存
    logic [7:0]     word_addr_h_r;      //字节地址高八位寄存
    logic [7:0]     word_addr_l_r;      //字节地址低八位寄存
    logic [7:0]     rdata_r;            //读数据寄存

    logic           num_word_addr_r;    //字节地址数寄存
    logic [7:0]     num_data_w_r;       //写数据个数寄存
    logic [7:0]     num_data_r_r;       //读数据个数寄存


    logic           scl_valid;          //scl有效
    logic [clogb2(SCL_CLK_CNT)-1:0] scl_cnt;            //scl计数器

    logic           scl_high;           //scl为高标志位
    logic           scl_low;            //scl为低标志位

    logic [7:0]     data_cnt;           //字节传输计数器


    //======================================================================
    //i2c scl生成

    //iic有效信号
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            scl_valid   <= #TCO '0;
        else if(done || error)
            scl_valid   <= #TCO '0;
        else if(wen || ren)
            scl_valid   <= #TCO '1;
        else
            scl_valid   <= #TCO scl_valid;
    end

    //scl分频计数
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            scl_cnt     <= #TCO '0;
        else if(scl_valid)
            if(scl_cnt == (SCL_CLK_CNT-1))
                scl_cnt <= #TCO '0;
            else
                scl_cnt <= #TCO scl_cnt + 1;
        else
            scl_cnt     <= #TCO '0;
    end

    //scl
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            scl         <= #TCO '1;
        else if(scl_cnt == (SCL_CLK_CNT>>1))
            scl         <= #TCO '0;
        else if(scl_cnt == '0)
            scl         <= #TCO '1;
        else
            scl         <= #TCO scl;
    end

    //scl为高
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            scl_high    <= #TCO '0;
        else if(scl_cnt == (SCL_CLK_CNT>>2)-1)
            scl_high    <= #TCO '1;
        else
            scl_high    <= #TCO '0;
    end

    //scl为低
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            scl_low     <= #TCO '0;
        else if(scl_cnt == (SCL_CLK_CNT>>1)+(SCL_CLK_CNT>>2))
            scl_low     <= #TCO '1;
        else
            scl_low     <= #TCO '0;
    end

    //======================================================================
    //i2c状态机
    enum logic[3:0] {
        IDLE        ,   //空闲状态
        READY       ,   //Ready状态
        W_START     ,   //写START
        R_START     ,   //读START
        D_ADDR_W    ,   //写操作Device Address
        D_ADDR_R    ,   //读操作Device Address
        W_ADDR      ,   //Word Address
        WR_DATA     ,   //写数据状态
        RD_DATA     ,   //读数据状态
        STOP            //停止状态
    }state,next_state;

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
            IDLE:   begin
                if(delay_cnt == SCL_CLK_CNT*2)
                    next_state  = READY;
                else
                    next_state  = IDLE;
            end
            READY:  begin
                if(wen || ren)
                    next_state  = W_START;
                else
                    next_state  = READY;
            end
            W_START:   begin
                if(~scl)
                    next_state  = D_ADDR_W;
                else
                    next_state  = W_START;
            end
            R_START:   begin
                if(scl_high)
                    next_state  = D_ADDR_R;
                else
                    next_state  = R_START;
            end
            D_ADDR_W:    begin
                if(data_cnt<17)
                    next_state  = D_ADDR_W;
                else 
                    if(scl_high)
                        if(sda_i == 0)
                            next_state  = W_ADDR;
                        else
                            next_state  = IDLE;
                    else
                        next_state  = D_ADDR_W;
            end
            D_ADDR_R:    begin
                if(data_cnt<17)
                    next_state  = D_ADDR_R;
                else 
                    if(scl_high)
                        if(sda_i == 0)
                            next_state  = RD_DATA;
                        else
                            next_state  = IDLE;
                    else
                        next_state  = D_ADDR_R;
            end
            W_ADDR:   begin 
                if(data_cnt<17)
                    next_state  = W_ADDR;
                else
                    if(scl_high)
                        if(sda_i == 0)
                            if(num_word_addr_r)
                                next_state  = W_ADDR;
                            else if(wr)
                                next_state  = WR_DATA;
                            else
                                next_state  = R_START;
                        else
                            next_state  = IDLE;
                    else
                        next_state  = W_ADDR;  
            end
            WR_DATA:    begin
                if(data_cnt<17)
                    next_state  = WR_DATA;
                else
                    if(scl_high)
                        if(sda_i == 0)
                            if(num_data_w_r > 0)
                                next_state  = WR_DATA;
                            else
                                next_state  = STOP;
                        else
                            next_state  = IDLE;
                else
                    next_state  = WR_DATA; 
            end
            RD_DATA:    begin
                if(data_cnt<17)
                    next_state  = RD_DATA;
                else
                    if(scl_high)
                        if(num_data_r_r > 0)            //ACK
                            next_state  = RD_DATA;
                        else                            //NOACK
                            next_state  = STOP;
                    else
                        next_state  = RD_DATA; 
            end
            STOP:       begin
                if(scl_high)
                    next_state  = IDLE;
                else
                    next_state  = STOP;
            end
            default:    begin
                next_state  = IDLE;
            end
        endcase
    end


    //======================================================================
    //控制信号和数据寄存
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)begin
            wen_r           <= #TCO '0;
            ren_r           <= #TCO '0;
            device_addr_r   <= #TCO '0;
            word_addr_h_r   <= #TCO '0;
            word_addr_l_r   <= #TCO '0;
        end
        else begin
            case(state)
                IDLE:begin
                    wen_r           <= #TCO '0;
                    ren_r           <= #TCO '0;
                    device_addr_r   <= #TCO '0;
                    word_addr_h_r   <= #TCO '0;
                    word_addr_l_r   <= #TCO '0;
                end 
                READY:begin 
                    wen_r           <= #TCO wen;
                    ren_r           <= #TCO ren;
                    device_addr_r   <= #TCO device_addr;
                    word_addr_h_r   <= #TCO word_addr_h;
                    word_addr_l_r   <= #TCO word_addr_l;
                end
                default:begin
                    wen_r           <= #TCO wen_r;
                    ren_r           <= #TCO ren_r;
                    device_addr_r   <= #TCO device_addr_r;
                    word_addr_h_r   <= #TCO word_addr_h_r;
                    word_addr_l_r   <= #TCO word_addr_l_r;
                end
            endcase    
        end            
    end

    assign  wr = wen_r ? 1 : 0;


    //======================================================================
    //状态机控制信号

    //延时计数
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            delay_cnt   <= #TCO '0;
        else
            case(state)
                IDLE:
                    if(delay_cnt == SCL_CLK_CNT*2)
                        delay_cnt   <= #TCO delay_cnt;
                    else
                        delay_cnt   <= #TCO delay_cnt + 1;
                default:
                    delay_cnt   <= #TCO '0;
            endcase
    end

    //字节计数器
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            data_cnt    <= #TCO '0;
        else if((state == D_ADDR_W)||
                (state == D_ADDR_R)||
                (state == W_ADDR)||
                (state == WR_DATA)||
                (state == RD_DATA))
            if(scl_high || scl_low)
                if(data_cnt == 8'd17)
                    data_cnt    <= #TCO '0;
                else
                    data_cnt    <= #TCO data_cnt+1;
            else
                data_cnt    <= #TCO data_cnt;
        else
            data_cnt    <= #TCO '0;
    end

    //字节地址宽度寄存
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            num_word_addr_r <= #TCO '0;
        else 
            case(state)
                READY:
                    if(wen || ren)
                        num_word_addr_r <= #TCO num_word_addr;
                    else
                        num_word_addr_r <= #TCO num_word_addr_r;
                W_ADDR:
                    if((data_cnt == 17) && scl_high && (sda_i == 0))
                        num_word_addr_r <= #TCO '0;
                    else
                        num_word_addr_r <= #TCO num_word_addr_r;
            default:
                num_word_addr_r <= #TCO num_word_addr;
            endcase
    end

    //写数据个数寄存
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            num_data_w_r    <= #TCO '0;
        else 
            case(state)
                READY:
                    if(wen)
                        num_data_w_r    <= #TCO num_data_w;
                    else
                        num_data_w_r    <= #TCO num_data_w_r;
                WR_DATA:
                    if((data_cnt == 17) && scl_high && (sda_i == 0) && (num_data_w_r > 0))
                        num_data_w_r    <= #TCO num_data_w_r - 1;
                    else
                        num_data_w_r    <= #TCO num_data_w_r;
                default:
                    num_data_w_r    <= #TCO num_data_w_r;
            endcase
    end

    //读数据个数寄存
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            num_data_r_r    <= #TCO '0;
        else 
            case(state)
                READY:
                    if(ren)
                        num_data_r_r    <= #TCO num_data_r;
                    else
                        num_data_r_r    <= #TCO num_data_r_r;
                RD_DATA:
                    if((data_cnt == 17) && scl_high && (num_data_r_r > 0))
                        num_data_r_r    <= #TCO num_data_r_r - 1;
                    else
                        num_data_r_r    <= #TCO num_data_r_r;
                default:
                    num_data_r_r    <= #TCO num_data_r_r;
            endcase
    end


    //======================================================================
    //i2c写

    //写有效
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            wvalid  <= '0;
        else
            case(state)
                WR_DATA:
                    if(data_cnt == 8'd17)
                        if((scl_high == '1)&&(sda_i == '0))
                            wvalid  <= '1;
                        else
                            wvalid  <= '0;
                    else
                        wvalid  <= '0;
                default:
                    wvalid  <= '0;
            endcase
    end


    //sda输出
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            sda_o       <= #TCO '1;
        else
            case(state)
                IDLE,READY:
                    sda_o   <= #TCO '1;
                W_START:
                    if(scl_high)
                        sda_o   <= #TCO '0;
                    else
                        sda_o   <= #TCO sda_o;
                R_START:
                    if(scl_high)
                        sda_o   <= #TCO '0;
                    else if(scl_low)
                        sda_o   <= #TCO '1;
                    else
                        sda_o   <= #TCO sda_o;
                D_ADDR_W:
                    if(scl_low)
                        case(data_cnt)
                            0:sda_o   <= #TCO device_addr_r[7];
                            2:sda_o   <= #TCO device_addr_r[6];
                            4:sda_o   <= #TCO device_addr_r[5];
                            6:sda_o   <= #TCO device_addr_r[4];
                            8:sda_o   <= #TCO device_addr_r[3];
                            10:sda_o  <= #TCO device_addr_r[2];
                            12:sda_o  <= #TCO device_addr_r[1];
                            14:sda_o  <= #TCO 1'b0;
                            default:sda_o   <= #TCO '0;
                        endcase
                    else
                        sda_o   <= #TCO sda_o;
                D_ADDR_R:
                    if(scl_low)
                        case(data_cnt)
                            0:sda_o   <= #TCO device_addr_r[7];
                            2:sda_o   <= #TCO device_addr_r[6];
                            4:sda_o   <= #TCO device_addr_r[5];
                            6:sda_o   <= #TCO device_addr_r[4];
                            8:sda_o   <= #TCO device_addr_r[3];
                            10:sda_o  <= #TCO device_addr_r[2];
                            12:sda_o  <= #TCO device_addr_r[1];
                            14:sda_o  <= #TCO 1'b1;
                            default:sda_o   <= #TCO '0;
                        endcase
                    else
                        sda_o   <= #TCO sda_o  ;
                W_ADDR:
                    if(scl_low)
                        case(data_cnt)
                            0:sda_o   <= #TCO num_word_addr_r ? word_addr_h_r[7] : word_addr_l_r[7];
                            2:sda_o   <= #TCO num_word_addr_r ? word_addr_h_r[6] : word_addr_l_r[6];
                            4:sda_o   <= #TCO num_word_addr_r ? word_addr_h_r[5] : word_addr_l_r[5];
                            6:sda_o   <= #TCO num_word_addr_r ? word_addr_h_r[4] : word_addr_l_r[4];
                            8:sda_o   <= #TCO num_word_addr_r ? word_addr_h_r[3] : word_addr_l_r[3];
                            10:sda_o  <= #TCO num_word_addr_r ? word_addr_h_r[2] : word_addr_l_r[2];
                            12:sda_o  <= #TCO num_word_addr_r ? word_addr_h_r[1] : word_addr_l_r[1];
                            14:sda_o  <= #TCO num_word_addr_r ? word_addr_h_r[0] : word_addr_l_r[0];
                            default:sda_o   <= #TCO '0;
                        endcase
                    else
                        sda_o   <= #TCO sda_o;
                WR_DATA:
                    if(scl_low)
                        case(data_cnt)
                            0:sda_o   <= #TCO wdata[7];
                            2:sda_o   <= #TCO wdata[6];
                            4:sda_o   <= #TCO wdata[5];
                            6:sda_o   <= #TCO wdata[4];
                            8:sda_o   <= #TCO wdata[3];
                            10:sda_o  <= #TCO wdata[2];
                            12:sda_o  <= #TCO wdata[1];
                            14:sda_o  <= #TCO wdata[0];
                            default:sda_o   <= #TCO '0;
                        endcase
                    else
                        sda_o   <= #TCO sda_o;
                RD_DATA:
                    if(|num_data_r_r)
                        sda_o   <= #TCO '0;
                    else if(data_cnt >= 8'd16)
                        sda_o   <= #TCO '1;
                    else
                        sda_o   <= #TCO sda_o;
                default:
                    sda_o   <= #TCO sda_o;
            endcase
    end

    //======================================================================
    //i2c读

    //读有效
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            rvalid  <= '0;
        else
            case(state)
                RD_DATA:
                    if(data_cnt == 8'd16)
                        if(scl_high || scl_low)
                            rvalid  <= '1;
                        else
                            rvalid  <= '0;
                    else
                        rvalid  <= '0;
                default:
                    rvalid  <= '0;
            endcase
    end

    //读数据串转并
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            rdata_r  <= #TCO '0;
        else
            case(state)
                RD_DATA:
                    if(scl_high)
                        case(data_cnt)
                            1: rdata_r <= #TCO {rdata_r[6:0],sda_i};
                            3: rdata_r <= #TCO {rdata_r[6:0],sda_i};
                            5: rdata_r <= #TCO {rdata_r[6:0],sda_i};
                            7: rdata_r <= #TCO {rdata_r[6:0],sda_i};
                            9: rdata_r <= #TCO {rdata_r[6:0],sda_i};
                            11:rdata_r <= #TCO {rdata_r[6:0],sda_i};
                            13:rdata_r <= #TCO {rdata_r[6:0],sda_i};
                            15:rdata_r <= #TCO {rdata_r[6:0],sda_i};
                            default:rdata_r <= #TCO rdata_r;
                        endcase
                    else
                        rdata_r <= #TCO rdata_r;
                default:
                    rdata_r <= #TCO rdata_r;
            endcase
    end

    //读输出
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            rdata   <= '0;
        else
            case(state)
                RD_DATA:
                    if(data_cnt >= 8'd16)
                        if(scl_high || scl_low)
                            rdata   <= rdata_r;
                        else
                            rdata   <= rdata;
                    else
                        rdata   <= rdata;
                default:
                    rdata   <= rdata;
            endcase
    end

    //======================================================================
    //i2c其他信号

    //sda输入输出使能
    always_ff@(posedge clk, negedge rst_n) begin
        if(!rst_n)
            sda_en  <= #TCO '0;
        else
            case(state)
                IDLE,READY:
                    sda_en  <= #TCO '0;
                W_START,R_START,STOP:
                    sda_en  <= #TCO '1;
                D_ADDR_W,D_ADDR_R,WR_DATA,W_ADDR:
                    if(data_cnt < 17)
                        sda_en  <= #TCO '1;
                    else
                        sda_en  <= #TCO '0;
                RD_DATA:
                    if(scl_low)
                        if(data_cnt < 16)
                            sda_en  <= #TCO '0;
                        else
                            sda_en  <= #TCO '1;
                    else
                        sda_en  <= #TCO sda_en;
                default:
                    sda_en  <= #TCO '0;
            endcase
    end

    //ready
    always_ff@(posedge clk, negedge rst_n) begin
        if(!rst_n)
            ready   <= #TCO '0;
        else
            case(state)
                READY:
                    ready   <= #TCO '1;
                default: 
                    ready   <= #TCO '0;
            endcase
    end


    //done
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            done    <= #TCO '0;
        else
            case(state)
                STOP:
                    if(scl_high)
                        done    <= #TCO '1;
                    else
                        done    <= #TCO done;
                default:
                    done    <= #TCO '0;
            endcase
    end

    //error
    always_ff@(posedge clk, negedge rst_n)begin
        if(!rst_n)
            error   <= #TCO '0;
        else
            case(state)
                D_ADDR_W:    begin
                    if(data_cnt<17)
                        error   <=  #TCO '0;
                    else 
                        if(scl_high)
                            if(sda_i == 0)
                                error   <=  #TCO '0;
                            else
                                error   <=  #TCO '1;
                        else
                            error   <=  #TCO '0;
                end
                D_ADDR_R:    begin
                    if(data_cnt<17)
                        error   <=  #TCO '0;
                    else 
                        if(scl_high)
                            if(sda_i == 0)
                                error   <=  #TCO '0;
                            else
                                error   <=  #TCO '1;
                        else
                            error   <=  #TCO '0;
                end
                W_ADDR:   begin 
                    if(data_cnt<17)
                        error   <=  #TCO '0;
                    else
                        if(scl_high)
                            if(sda_i == 0)
                                error   <=  #TCO '0;
                            else
                                error   <=  #TCO '1;
                        else
                            error   <=  #TCO '0;
                end
                WR_DATA:    begin
                    if(data_cnt<17)
                        error   <=  #TCO '0;
                    else
                        if(scl_high)
                            if(sda_i == 0)
                                error   <=  #TCO '0;
                            else
                                error   <=  #TCO '1;
                    else
                        error   <=  #TCO '0;
                end
                default:    begin
                    error   <=  #TCO '0;
                end
            endcase
    end

endmodule
