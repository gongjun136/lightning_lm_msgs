#ifndef _VIDEO_ENCODE_API_H
#define _VIDEO_ENCODE_API_H

#if defined(_WIN32) || defined(_WIN64)
#ifndef CALLBACK
#define CALLBACK __stdcall
#endif
#ifdef  VIDEOENC_EXPORTS
#define VIDEOENC_API extern "C" __declspec(dllexport)
#else
#define VIDEOENC_API extern "C" __declspec(dllimport)
#endif
#elif defined (__linux__)
#define VIDEOENC_API extern "C"
#ifndef CALLBACK
#define CALLBACK 
#endif
#else 
#define VIDEOENC_API
#ifndef CALLBACK
#define CALLBACK 
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif


/*
type:数据类型,0:其它，1:关键帧
data:编码后的数据
datalen:编码后的数据长度
userdata：用户数据
*/
typedef  void (CALLBACK *VIDEOENC_CALLBACK)(unsigned char type,unsigned char *data,int datalen,void *userdata);

//编码参数
typedef struct tagVIDEOENC_PARA
{
        unsigned char           encfmt;                                         //编码格式，0:H264,1:H265          
        unsigned char           fps;                                                //帧率                                                  
        unsigned char            pixfmt;                                        //pixfmt:像素格式，0:YUV420P,  1:YUYV 
        unsigned char            dma;                                            //是否使用dma  
        unsigned char            insert_sps_pps_at_idr;     //每个idr帧插入sps和pps 
        unsigned char            alliframe;                                 //全部iframe编码  
        unsigned char            res[10];      
        unsigned int                bitrate;                                     //比特率                                 
        unsigned int                ifi;                                                //I帧间隔                                                                
        unsigned int                width;                                      //宽度                  
        unsigned int                height;                                     //高度
        unsigned int                bufs;                                       //缓存帧数
}VIDEOENC_PARA;

//视频编码帧信息
typedef struct tagVIDEOENC_FRAME_INFO
{
      unsigned int       sec;           
      unsigned int       nan;
      unsigned int       exposure;    //us
      unsigned int       res[4];
}VIDEOENC_FRAME_INFO;

/*
        function:初始化
        return:成功0，失败-1
*/
VIDEOENC_API int CALLBACK  VIDEOENC_Init();

/*
        function:反初始化
        return:成功0，失败-1
*/

VIDEOENC_API int CALLBACK VIDEOENC_Release();

/*
        function:创建句柄
        para:编码参数
        return:成功返回句柄，失败返回NULL
*/
VIDEOENC_API void * CALLBACK VIDEOENC_CreateHandle(VIDEOENC_PARA *para);

/*
        function:销毁句柄
        handle：句柄
       return:成功0，失败-1
*/
VIDEOENC_API int CALLBACK VIDEOENC_DestroyHandle(void *handle);

/*
        function:设置数据回调
        handle：句柄
        callback：回调函数，接收编码后的数据
        userdata：用户数据
       return:成功0，失败-1
*/
VIDEOENC_API int CALLBACK VIDEOENC_SetDataCallBack(void *handle,VIDEOENC_CALLBACK callback,void *userdata);

/*
        function:塞入数据
        handle：句柄
        data：数据
        datalen：数据长度
        timeout：超时时间,单位ms
       return:成功0，失败-1
*/
VIDEOENC_API int CALLBACK VIDEOENC_InputData(void *handle,unsigned char *data,int datalen,int timeout);

/*
        function:塞入数据
        handle：句柄
        data：数据
        datalen：数据长度
         timeout：超时时间,单位ms
       return:成功0，失败-1
*/
VIDEOENC_API int CALLBACK VIDEOENC_InputData2(void *handle,VIDEOENC_FRAME_INFO *fi,unsigned char *data,int datalen,int timeout);

/*
        function:设置比特率，可以在编码过程中调用
        handle：句柄
        bitrate：比特率
       return:成功0，失败-1
*/
VIDEOENC_API int CALLBACK VIDEOENC_SetBitrate(void *handle,unsigned int bitrate);
#ifdef __cplusplus
}
#endif
#endif
