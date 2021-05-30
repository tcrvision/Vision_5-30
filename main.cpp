/***
 * When I wrote this, only God and I understood what I was doing
 * Now, God only knows
 *                    _ooOoo_
 *                   o8888888o
 *                   88" . "88
 *                   (| -_- |)
 *                    O\ = /O
 *                ____/`---'\____
 *              .   ' \\| |// `.
 *               / \\||| : |||// \
 *             / _||||| -:- |||||- \
 *               | | \\\ - /// | |
 *             | \_| ''\---/'' | |
 *              \ .-\__ `-` ___/-. /
 *           ___`. .' /--.--\ `. . __
 *        ."" '< `.___\_<|>_/___.' >'"".
 *       | | : `- \`.;`\ _ /`;.`/ - ` : | |
 *         \ \ `-. \_ __\ /__ _/ .-` / /
 * ======`-.____`-.___\_____/___.-`____.-'======
 *                    `=---='
 *.............................................
 *          佛祖保佑             永无BUG
 *
 **/
#include "buff_detect.h"
#include<thread>
#include "thread_control.h"
#include <unistd.h>
int main() {
    ThreadControl ImageControl;
    thread produce_task(&ThreadControl::ImageProduce,&ImageControl);
    thread process_task(&ThreadControl::ImageProcess, &ImageControl);
#ifdef GET_DATA_THREAD
    std::thread receive_task(&ThreadControl::GetData,&ImageControl);
#endif
#ifdef SAVE_VIDEO_THREAD
    std::thread write_task(&ThreadControl::ImageWrite, &ImageControl);
#endif
    produce_task.join();
    process_task.join();
#ifdef GET_DATA_THREAD
    receive_task.detach();
#endif
#ifdef SAVE_VIDEO_THREAD
    write_task.join();
#endif
    return 1;
}