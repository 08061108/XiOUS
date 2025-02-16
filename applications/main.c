/* --JR
一个基于RT-Thread实时操作系统的STM32F407开发板的应用程序入口 初始化系统框架、配置LED引脚，并使LED以特定频率闪烁
*/


/*
 * @Author: chunyexixiaoyu
 * @Date: 2021-09-24 16:33:15
 * @LastEditTime: 2021-09-24 15:48:30
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \xiuos\Ubiquitous\RT_Thread\bsp\stm32f407-atk-coreboard\applications\main.c
 */

/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

/* --JR 
<rtthread.h>--RT-Thread操作系统的核心头文件，包含线程管理、调度等基本功能 
<board.h>--与具体开发板相关的配置和定义 
<stdio.h>和<string.h>--标准输入输出和字符串处理函数
#ifdef RT_USING_POSIX--如果RT_USING_POSIX宏已经被定义，则包含
  <pthread.h>--POSIX线程（POSIX Threads）相关的函数和数据结构的定义
  <unistd.h>--许多与UNIX标准相关的函数定义，比如read、write函数用于进行基本的文件I/O操作
  <dfs_poll.h>--包含与轮询文件描述符状态相关的函数或结构体定义，用于处理多个文件描述符的事件等待和通知机制
  <dfs_posix.h>--针对POSIX兼容的文件系统操作的定义，使得在RT - Thread系统中能够使用类似POSIX的文件系统操作方式
  <dfs.h>--分布式文件系统的总体头文件，包含了文件系统的基本操作、数据结构和全局配置等相关定义
  #ifdef RT_USING_POSIX_TERMIOS--如果定义了RT_USING_POSIX_TERMIOS宏，就会包含
    <posix_termios.h>--包含了POSIX终端控制接口的定义，用于配置和控制终端设备的属性，如波特率、数据位、停止位、奇偶校验等，在涉及到串口通信或者与终端交互的场景中非常有用
*/

#include <rtthread.h>
#include <board.h>
#include <stdio.h>
#include <string.h>
#ifdef RT_USING_POSIX
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <dfs_poll.h>
#include <dfs_posix.h>
#include <dfs.h>
#ifdef RT_USING_POSIX_TERMIOS
#include <posix_termios.h>
#endif
#endif

/* --JR
LED0_PIN--使用GET_PIN宏定义LED0连接的引脚，这里是GPIO(General-Purpose Input/Output)G的第15个引脚
FrameworkInit()--声明一个外部函数，用于初始化系统框架
  extern关键字--用于告诉编译器该函数在其他地方定义，当前文件只是引用
*/
#define LED0_PIN    GET_PIN(G, 15)
extern int FrameworkInit();


/* --JR
rt_pin_mode--将LED0连接的引脚（GPIOG第15引脚）PIN_MODE_OUTPUT--设置为输出模式
rt_thread_mdelay--主线程延时100毫秒，确保引脚模式设置完成 (阻塞式的延时函数-意味着在延时期间，线程不会执行其他任务)
printf("XIUOS stm32f4 build %s %s\n",__DATE__,__TIME__); --在串口或其他输出设备上打印当前程序的编译日期和时间
  XIUOS stm32f4 build--输出的前缀信息，说明这是为 "XIUOS stm32f4" 项目构建的信息
  %s--格式说明符，表示一个字符串 会被后面提供的相应参数替换
  __DATE__--格式通常为 "Mmm dd yyyy"，例如 "Sep 24 2021"  对应前面的%s
  __TIME__--格式通常为 "hh:mm:ss"，例如 "16:33:15" 对应前面另一个%s
--开发者可以清楚地知道程序是何时构建的，有助于版本管理和问题排查
while (count++)--进入一个无限循环，使LED0以500毫秒的高电平和500毫秒的低电平交替变化，实现闪烁效果
*/
int main(void)
{
     int count = 1;
     rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
     rt_thread_mdelay(100);
     FrameworkInit();
     printf("XIUOS stm32f4 build %s %s\n",__DATE__,__TIME__);          
     while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}

