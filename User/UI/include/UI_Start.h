//UI_Start.h

#ifndef __UI_START_H
#define __UI_START_H

/*
 * 函数功能：启动页等待约 4 秒，期间达到定标按钮点击次数就进入脚踏定标页。
 * 输入参数：无。
 * 返回参数：无；未进入定标时，等待结束后返回。
 */
void UI_Start_Fun(void);

/*
 * 函数功能：切到主运行页，并隐藏旧报警图片。
 * 输入参数：无。
 * 返回参数：无。
 */
void UI_Show_init(void);

#endif  //__UI_START_H


