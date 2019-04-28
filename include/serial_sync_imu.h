#ifndef _SERIAL_SYNC_IMU_H_
#define _SERIAL_SYNC_IMU_H_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <thread>
#include <Eigen/Core>
#ifdef _WIN32
    #include <windows.h>
#else
    #include <termios.h>
    #include <unistd.h>
#endif
#include "concurrency.h"

#define BUFF_SIZE 100
#define IMU_HEADR '$'
#define IMU_FOOTR '@'
#define IMG_HEADR '#'
#define IMG_FOOTR '&'

namespace ark {

    #define IMU_DATA_SIZE 20
    struct ImuDataRaw{
        uint32_t timestamp,id;
        int16_t ax,ay,az,gx,gy,gz;
    };//ImuDataRaw

    #define IMG_DATA_SIZE 8
    struct ImgTimeInfo{
        uint32_t timestamp,id;
    };//ImgTimeInfo

    struct ImuParams
    {
        double rate; //in frames per second : 200 for MPU6050
        double gyro_div; //conversion from raw gyro measurement to rad/s  : 1/16.4 *pi/180 for MPU6050
        double accel_div; //conversion from raw accel measurement to m/s^z : 1/8192 *9.81 for MPU6050
        //noise params
        //any other relavent info s/a conversion info
    };//ImuParams

    class SerialSyncImu{

    public:

        volatile long frame_count_;
        SerialSyncImu(char* port_addr, const ImuParams* params):
        params_(params){
            running_ = false;
            #ifdef _WIN32
                port_=CreateFile("COM3", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
                DCB dcbSerialParams = { 0 }; // Initializing DCB structure
                dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
                if(!GetCommState(port_, &dcbSerialParams))
                    std::cout << "ERROR GETTING COM PORT" << std::endl;
                dcbSerialParams.BaudRate = CBR_115200;  // Setting BaudRate = 9600
                dcbSerialParams.ByteSize = 8;         // Setting ByteSize = 8
                dcbSerialParams.StopBits = ONESTOPBIT;// Setting StopBits = 1
                dcbSerialParams.Parity   = NOPARITY;  // Setting Parity = None
                SetCommState(port_, &dcbSerialParams);
            #else
                port_=open(port_addr, O_RDWR | O_NOCTTY | O_NDELAY);
                fcntl(port_, F_SETFL,0);
            #endif
        }

        bool start(){
            char buf[1];
            buf[0]='*';
            #ifdef _WIN32
                WriteFile(port_,buf,1,0,0);
            #else
                write(port_,buf,1);
            #endif
            if(running_==false){
                running_=true;
                run_thread_ = std::thread(&SerialSyncImu::run,this);
                return true;
            }
            return false;
        }

        bool stop(){
            if(running_==true){
                running_ =false;
                run_thread_.join();
                return true;
            }
            return false;
        }

        ~SerialSyncImu(){
            if(running_){
                running_=false;
                run_thread_.join();
            }
        }

        bool running(){
            return running_;
        }

        bool getImuToTime(double timestamp, std::vector<ImuPair>& data_out){
            ImuDataRaw imu_data;
            imu_data.timestamp=0;
            while((imu_data.timestamp+1000.0/params_->rate)*1e6<timestamp){
                //convert ImuDataRaw to ImuPair
                if(imu_queue_.try_dequeue(&imu_data)){
                    ImuPair imu_data_out = convert(imu_data);
                    data_out.push_back(imu_data_out);
                }
            }
            if((imu_data.timestamp+1000.0/params_->rate)*1e6<timestamp)
                return false;
            return true;

        };

        double getFrameTimestamp(long frame_num){
            ImgTimeInfo img_time;
            img_time.id=0;
            while(img_time.id<frame_num){
                if(img_queue_.try_get_front(&img_time))
                    if(img_time.id>frame_num)
                        return -1;
                img_queue_.dequeue();
            }
            return double(img_time.timestamp*1e6); //convert to nanoseconds
        };


    private:

        ImuPair convert(const ImuDataRaw& imu_in){
             ImuPair imu_out { double(imu_in.timestamp)*1e6, //convert to nanoseconds
                    Eigen::Vector3d(imu_in.gx/params_->gyro_div,imu_in.gy/params_->gyro_div,imu_in.gz/params_->gyro_div),
                    Eigen::Vector3d(imu_in.ax/params_->accel_div,imu_in.ay/params_->accel_div,imu_in.az/params_->accel_div)};
            return imu_out;
        }

        void parse_buf(const char* buff_in, int bytes, char* buff_out, int& bytes_out){
            int it=0;
            for(; it<bytes; it++){
                if(buff_in[it]==IMU_HEADR){
                    if(it+IMU_DATA_SIZE+1>=bytes)
                        break;
                    it++;
                    ImuDataRaw new_imu;
                    new_imu.timestamp=*(uint32_t*)(buff_in+it);
                    it+=sizeof(uint32_t);
                    new_imu.id=*(uint32_t*)(buff_in+it);
                    it+=sizeof(uint32_t);
                    new_imu.ax=*(int16_t*)(buff_in+it);
                    it+=sizeof(int16_t);
                    new_imu.ay=*(int16_t*)(buff_in+it);
                    it+=sizeof(int16_t);
                    new_imu.az=*(int16_t*)(buff_in+it);
                    it+=sizeof(int16_t);
                    new_imu.gx=*(int16_t*)(buff_in+it);
                    it+=sizeof(int16_t);
                    new_imu.gy=*(int16_t*)(buff_in+it);
                    it+=sizeof(int16_t);
                    new_imu.gz=*(int16_t*)(buff_in+it);
                    it+=sizeof(int16_t);
                    if(buff_in[it]==IMU_FOOTR){
                        imu_queue_.enqueue(new_imu);
                        //std::cout << "IMU: " << new_imu.timestamp << " , " 
                        //    << new_imu.id << " , " << new_imu.ax << " , " 
                        //    << new_imu.ay << " , " << new_imu.az << " , " 
                        //    << new_imu.gx << " , " << new_imu.gy << " , "        
                        //    << new_imu.gz << std::endl;
                    }else{
                        it-=IMU_DATA_SIZE+1;
                    }
                }
                if(buff_in[it]==IMG_HEADR){
                    if(it+IMG_DATA_SIZE+1>=bytes)
                        break;
                    it++;
                    ImgTimeInfo new_img_time;
                    new_img_time.timestamp=*(uint32_t*)(buff_in+it);
                    it+=sizeof(uint32_t);
                    new_img_time.id=*(uint32_t*)(buff_in+it);
                    it+=sizeof(uint32_t);
                    if(buff_in[it]==IMG_FOOTR){
                        if(new_img_time.id < frame_count_)
                            img_queue_.clear();
                        frame_count_=new_img_time.id;
                        img_queue_.enqueue(new_img_time);
                        //std::cout << "IMG: " << new_img_time.timestamp << " , " 
                        //    << new_img_time.id << std::endl;
                    }else{
                        it-=IMG_DATA_SIZE+1;
                    }
                }
            }
            bytes_out=bytes-it;
            memcpy(buff_out, buff_in+it,bytes_out);

        }

        void run(){
            char buff[BUFF_SIZE], buff_rem[BUFF_SIZE], buff_total[BUFF_SIZE*2];
            #ifdef _WIN32
                DWORD bytes;
            #else
                int bytes;
            #endif
            while(running_){
                #ifdef _WIN32
                    ReadFile(port_,buff,BUFF_SIZE,&bytes,0);
                #else
                    bytes=read(port_,buff,BUFF_SIZE);
                #endif
                int bytes_rem;
                parse_buf(buff,bytes,buff_rem,bytes_rem);
                while(bytes<BUFF_SIZE){
                    #ifdef _WIN32
                        ReadFile(port_,buff,BUFF_SIZE,&bytes,0);
                    #else
                        bytes=read(port_,buff,BUFF_SIZE);
                    #endif
                    memcpy(buff_total,buff_rem,bytes_rem);
                    memcpy(buff_total+bytes_rem,buff,bytes);
                    parse_buf(buff_total,bytes+bytes_rem,buff_rem,bytes_rem);
                }
            }

            #ifdef _WIN32
                CloseHandle(port_);
            #else
                close(port_);
            #endif

        }

        #ifdef _WIN32
            HANDLE port_;
        #else
            int port_;
        #endif
        volatile bool running_;
        single_consumer_queue<ImuDataRaw> imu_queue_;
        single_consumer_queue<ImgTimeInfo> img_queue_;
        std::thread run_thread_;
        const ImuParams* params_;

    };//SerialSyncImu

}//ark

#endif //_SERIAL_SYNC_IMU_H_