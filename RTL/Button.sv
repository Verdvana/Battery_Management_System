//=============================================================================
//
//Module Name:					Button.sv
//Department:					Xidian University
//Function Description:	        按键控制器
//
//------------------------------------------------------------------------------
//
//Version 	Design		Coding		Simulata	  Review		Rel data
//V1.0		Verdvana	Verdvana	Verdvana		  			2020-09-26
//
//------------------------------------------------------------------------------
//
//Version	Modified History
//V1.0		软件滤波，延时参数可调
//          上升沿检测，保持一个周期	
//
//=============================================================================

`timescale 1ns/1ps

module Button #(
    parameter   BUTTON_WIDTH    = 2                 ,   //按键位宽
                SYS_CLOCK       = 100_000_000       ,   //时钟频率
                FILTER_DELAY    = 5                     //消抖延迟（单位：ms）
)(
    input  logic                    clk             ,   //时钟
    input  logic                    rst_n           ,   //异步复位

    input  logic [BUTTON_WIDTH-1:0] button_in       ,   //按键输入

    output logic [BUTTON_WIDTH-1:0] button_out      ,   //消抖后输出
    output logic [BUTTON_WIDTH-1:0] button_edge         //消抖后输出一个周期
);

    parameter   TCO = 1,                                //寄存器延迟
                NUM = SYS_CLOCK/1000*FILTER_DELAY;      //消抖计数器值
    
    //位宽计算函数
    function integer clogb2 (input integer depth);
    begin
        for (clogb2=0; depth>0; clogb2=clogb2+1) 
            depth = depth >>1;                          
    end
    endfunction

    logic [clogb2(NUM)-1:0]     cnt         [BUTTON_WIDTH]; //消抖计数器
    logic                       flag_out    [BUTTON_WIDTH]; //输出flag
    logic                       flag_edge   [BUTTON_WIDTH]; //边沿flag

    genvar k;
    generate for (k=0;k<BUTTON_WIDTH;k++)
    begin:fliter
        always_ff@(posedge clk, negedge rst_n)begin
            if(!rst_n)
                cnt[k]  <= #TCO '0;
            else if(button_in[k])
                if(cnt[k] == NUM-1)
                    cnt[k]  <= #TCO cnt[k];
                else
                    cnt[k]  <= #TCO cnt[k] + 1;
            else
                cnt[k]  <= #TCO '0;
        end

        always_ff@(posedge clk, negedge rst_n)begin
            if(!rst_n)
                flag_out[k] <= #TCO '0;
            else if(cnt[k]  == NUM-1)
                flag_out[k] <= #TCO '1;
            else
                flag_out[k] <= #TCO '0;
        end 

        always_ff@(posedge clk, negedge rst_n)begin
            if(!rst_n)
                flag_edge[k] <= #TCO '0;
            else if(flag_out[k]=='0)
                if(cnt[k]  == NUM-1)
                    flag_edge[k] <= #TCO '1;
                else
                    flag_edge[k] <= #TCO '0;
            else
                flag_edge[k] <= #TCO '0;
        end 

        assign  button_out[k]   = flag_out[k];
        assign  button_edge[k]  = flag_edge[k];
    end
    endgenerate


endmodule