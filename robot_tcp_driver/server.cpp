
#include <sever.h>
#include <fstream>

using namespace std;

int i=0;
u_char cmdReceiveData[256]={};
u_char cmdSendData[256]={};
int port=8700;
bool runFlag;
int forClientSockfd = 0;
int sockfd = 0;       //sockfd套接字
int recvbytes=0;                          //判断是否断开连接
struct sockaddr_in serverInfo,clientInfo; // clientInfo连接实体地址
socklen_t addrlen = sizeof(clientInfo);

void server();
CmdMessage *Receive();

std::vector<CmdMessage> buffer_pool_;

CmdMessage *recv_container_ptr_;


bool SeverInit()
{
    /**
 * 1.创建套接字
 */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    /**
     * 2.绑定
     */
    bzero(&serverInfo, sizeof(serverInfo));
    serverInfo.sin_family = PF_INET;
    serverInfo.sin_addr.s_addr = INADDR_ANY;
    serverInfo.sin_port = htons(port);
    bind(sockfd, (struct sockaddr *) &serverInfo, sizeof(serverInfo));
    /**
     * 3.监听
     */
    listen(sockfd,5);

    recv_container_ptr_=new CmdMessage();

    thread t1(server);
    t1.detach();



    return true;

}

void server() {

    ros::Rate server_loop_rate(50);
    while (1) {
        ROS_INFO("Waiting Client...");
        forClientSockfd = accept(sockfd, (struct sockaddr *) &clientInfo, &addrlen);
        ROS_INFO("Client connect");
        runFlag=true;
        ros::Rate receive_loop_rate(1000);
        while(runFlag) {
            CmdMessage *container_ptr=Receive();
            if(container_ptr)
            {
                buffer_pool_.push_back(*container_ptr);
            }
            //usleep(100);
            receive_loop_rate.sleep();
        }
        server_loop_rate.sleep();
        //usleep(10000);
    }

}

CmdMessage *Receive() {
    //read the file
    //ROS_INFO("run Receive");
    char head_temp[sizeof(CmdHead)] = {};
    CmdMessage *recv_container = recv_container_ptr_;
    memset(head_temp, 0, sizeof(head_temp));
    recvbytes = recv(forClientSockfd, head_temp, sizeof(head_temp), 0);


    //!异常情况退出
    if (recvbytes == 0 || recvbytes == -1) {
        ROS_INFO("Receive error 1!!!");
        runFlag = false;
        return nullptr;
        //return 0;
    }
    recv_container->head.sof = head_temp[0];
    recv_container->head.id = head_temp[1];
    recv_container->head.len = head_temp[2];
    if (recv_container->head.sof == HEAD_SOF){// head check
        recvbytes = recv(forClientSockfd, cmdReceiveData, recv_container->head.len + 1, 0); //receive all data and frame tail
        if (recvbytes == 0 || recvbytes == -1) {
            ROS_INFO("Receive error 2!!!");
            runFlag = false;
            return nullptr;
        }
        recv_container->tail = cmdReceiveData[recv_container->head.len];// frame tail
        if (recv_container->tail == HEAD_TAIL){// tail check
            cmdReceiveData[recv_container->head.len]=0x00; //frame tail clean;
            memcpy(recv_container->data, cmdReceiveData, recv_container->head.len);
            //ROS_INFO("Receive *%d* CmdMessage success  !!!",recv_container->head.id);
        }else{
            ROS_INFO("Receive frame tail error !!!");
            ROS_INFO("cmdReceiveData:");
            for(int i=0;i<recv_container->head.len + 1;i++)ROS_INFO("|%d|",cmdReceiveData[i]);
            return nullptr;
        }
    }else{
        ROS_INFO("Receive frame head error !!!");
        for(int i=0;i<sizeof(CmdHead);i++)ROS_INFO("|%d|",head_temp[i]);
        return nullptr;
    }

    return recv_container;

}



bool Take(CmdMessage *recv_container)
{
    if(!buffer_pool_.size())return false;
    else{
        //ROS_INFO("recv_container SIZE %d",(int)buffer_pool_.size());
        *recv_container=buffer_pool_.front();
        buffer_pool_.erase(buffer_pool_.begin());
        return true;
    }
}

bool Send(CmdMessage &send_container)
{
    send_container.head.sof=HEAD_SOF;
    send_container.tail=HEAD_TAIL;
    cmdSendData[0]=send_container.head.sof;
    cmdSendData[1]=send_container.head.id;
    cmdSendData[2]=send_container.head.len;
    if(send_container.head.id==CMD_SEND_MAP){ //if send map data
        cmdSendData[3]=send_container.tail;
        MapHead *map_head=new MapHead(); // build map head use to compute map data lenth
        memcpy(map_head,send_container.data_ptr,sizeof(MapHead));
        send(forClientSockfd, cmdSendData,sizeof(CmdHead), 0); //send cmd head
        send(forClientSockfd, send_container.data_ptr,map_head->height.at * map_head->width.at + sizeof(MapHead), 0); //send map message
        send(forClientSockfd, cmdSendData + sizeof(CmdHead), sizeof(uint8_t), 0);//send cmd tail
        ROS_INFO("CmdMessage |%d| Map success!!",send_container.head.id);
    }
    else{//if send cmd data
        cmdSendData[send_container.head.len + sizeof(CmdHead)]=send_container.tail;
        memcpy(cmdSendData + sizeof(CmdHead),send_container.data,send_container.head.len);
        //for(int i=0;i<send_container.head.len;i++)ROS_INFO("|%d|",send_container.data[i]); debug data
        send(forClientSockfd, cmdSendData,send_container.head.len + sizeof(CmdHead) + sizeof(uint8_t) , 0);
        ROS_INFO("CmdMessage |%d| data success!!",send_container.head.id);
    }


      return true;
}







