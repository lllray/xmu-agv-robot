/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "protocol.h"
//LX ADD 0325
#include "protocol_define.h"
//END 0325

#include <iomanip>

namespace roborts_sdk {

Protocol::Protocol(std::shared_ptr<SerialDevice> serial_device_ptr) :
    running_(false),
    serial_device_ptr_(serial_device_ptr), seq_num_(0),
    is_large_data_protocol_(false), reuse_buffer_(true),
    poll_tick_(10) {

}

Protocol::~Protocol() {
  running_ = false;
  if (send_poll_thread_.joinable()) {
    send_poll_thread_.join();
  }
  if (receive_pool_thread_.joinable()) {
    receive_pool_thread_.join();
  }
  if (recv_stream_ptr_) {
    delete[] recv_stream_ptr_->recv_buff;
    delete recv_stream_ptr_;
  }
  if (recv_buff_ptr_) {
    delete[]recv_buff_ptr_;
  }
  if (recv_container_ptr_) {
    delete recv_container_ptr_;
  }
}

bool Protocol::Init() {

  seq_num_ = 0;
  auto max_buffer_size = BUFFER_SIZE;
  auto max_pack_size = MAX_PACK_SIZE;
  auto session_table_num = SESSION_TABLE_NUM;
  memory_pool_ptr_ = std::make_shared<MemoryPool>(max_buffer_size,
                                                  max_pack_size,
                                                  session_table_num);
  memory_pool_ptr_->Init();

  recv_buff_ptr_ = new uint8_t[BUFFER_SIZE];

  recv_stream_ptr_ = new RecvStream();
  recv_stream_ptr_->recv_buff = new uint8_t[MAX_PACK_SIZE];
  recv_stream_ptr_->recv_index = 0;
  recv_stream_ptr_->reuse_index = 0;
  recv_stream_ptr_->reuse_count = 0;

  recv_container_ptr_ = new RecvContainer();

  SetupSession();

  running_ = true;
    LOG_INFO<<"HEADER_LEN:" <<sizeof(Header);
  send_poll_thread_ = std::thread(&Protocol::AutoRepeatSendCheck, this);

  receive_pool_thread_ = std::thread(&Protocol::ReceivePool, this);
  return true;
}

void Protocol::AutoRepeatSendCheck() {
  while (running_) {
    unsigned int i;

    std::chrono::steady_clock::time_point current_time_stamp;

    for (i = 1; i < SESSION_TABLE_NUM; i++) {

      if (cmd_session_table_[i].usage_flag == 1) {
        current_time_stamp = std::chrono::steady_clock::now();
        if ((std::chrono::duration_cast<std::chrono::milliseconds>
            (current_time_stamp - cmd_session_table_[i].pre_time_stamp) >
            cmd_session_table_[i].ack_timeout)) {

          memory_pool_ptr_->LockMemory();
          if (cmd_session_table_[i].retry_time > 0) {

            if (cmd_session_table_[i].sent >= cmd_session_table_[i].retry_time) {
              LOG_ERROR << "Sending timeout, Free session "
                         << static_cast<int>(cmd_session_table_[i].session_id);
              FreeCMDSession(&cmd_session_table_[i]);
            } else {
              LOG_ERROR << "Retry session "
                         << static_cast<int>(cmd_session_table_[i].session_id);
              DeviceSend(cmd_session_table_[i].memory_block_ptr->memory_ptr);
              cmd_session_table_[i].pre_time_stamp = current_time_stamp;
              cmd_session_table_[i].sent++;
            }
          } else {
            DLOG_ERROR << "Send once " << i;
            DeviceSend(cmd_session_table_[i].memory_block_ptr->memory_ptr);
            cmd_session_table_[i].pre_time_stamp = current_time_stamp;
          }
          memory_pool_ptr_->UnlockMemory();
        } else {
//        DLOG_INFO<<"Wait for timeout Session: "<< i;
        }
      }
    }
    usleep(1000);
  }
}

void Protocol::ReceivePool() {
  while (running_) {
      //LOG_INFO<<"run ReceivePool";
    RecvContainer *container_ptr = Receive();
     // LOG_INFO<<"run container_ptr="<<container_ptr;
    if (container_ptr) {
      //LOG_INFO<<"run container_ptr==ture";
      if (buffer_pool_map_.count(std::make_pair(container_ptr->command_info.cmd_set,
                                                container_ptr->command_info.cmd_id)) == 0) {
        buffer_pool_map_[std::make_pair(container_ptr->command_info.cmd_set,
                                        container_ptr->command_info.cmd_id)]
            = std::make_shared<CircularBuffer<RecvContainer>>(100);

        LOG_INFO<<"Capture command: "
                <<"cmd set: 0x"<< std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.cmd_set)
                <<", cmd id: 0x"<< std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.cmd_id)
                <<", sender: 0x"<< std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.sender)
                <<", receiver: 0x" <<std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.receiver);

      }
      //1 time copy
      buffer_pool_map_[std::make_pair(container_ptr->command_info.cmd_set,
                                      container_ptr->command_info.cmd_id)]->Push(*container_ptr);
    }

    usleep(100);
  }
}

bool Protocol::Take(const CommandInfo *command_info,
                    MessageHeader *message_header,
                    void *message_data) {
  //no data
  if (buffer_pool_map_.count(std::make_pair(command_info->cmd_set,
                                            command_info->cmd_id)) == 0) {
//    DLOG_ERROR<<"take failed";
    return false;
  } else {
    //1 time copy
    RecvContainer container;
    if (!buffer_pool_map_[std::make_pair(command_info->cmd_set,
                                         command_info->cmd_id)]->Pop(container)) {
//      DLOG_EVERY_N(ERROR, 100)<<"nothing to take";
      return false;
    }

    bool mismatch = false;

      if (int(container.command_info.receiver) != int(command_info->receiver)){
        DLOG_ERROR << "Requested receiver: " << std::setw(2) << std::hex << std::setfill('0')
                   << int(container.command_info.receiver)
                   << ", Get receiver: " << std::setw(2) << std::hex << std::setfill('0')
                   << int(container.command_info.receiver);
        mismatch = true;
      }

      if (int(container.command_info.sender) != int(command_info->sender)){
        DLOG_ERROR << "Requested sender: " << std::setw(2) << std::hex << std::setfill('0') << int(command_info->sender)
                   << ", Get sender: " << std::setw(2) << std::hex << std::setfill('0')
                   << int(container.command_info.sender);
        mismatch = true;
      }
    if (int(container.command_info.length) !=int(command_info->length)){
      DLOG_ERROR << "Requested length: "<< int(command_info->length)
                <<", Get length: "<< int(container.command_info.length);
      mismatch = true;
    }
    if(mismatch){
      buffer_pool_map_[std::make_pair(command_info->cmd_set,
                                      command_info->cmd_id)]->Push(container);
      return false;
    }

    //1 time copy
    memcpy(message_header, &(container.message_header), sizeof(message_header));
    memcpy(message_data, &(container.message_data), command_info->length);

    return true;
  }
}

bool Protocol::SendResponse(const CommandInfo *command_info,
                            const MessageHeader *message_header,
                            void *message_data) {
  //LOG_INFO<<"run SendResponse!";
  return SendACK(message_header->session_id,
                 message_header->seq_num,
                 command_info->receiver,
                 message_data, command_info->length);
}
bool Protocol::SendRequest(const CommandInfo *command_info,
                           MessageHeader *message_header,
                           void *message_data) {
  //LOG_INFO<<"run SendRequest!";
  return SendCMD(command_info->cmd_set, command_info->cmd_id,
                 command_info->receiver, message_data, command_info->length,
                 CMDSessionMode::CMD_SESSION_AUTO, message_header);
}

bool Protocol::SendMessage(const CommandInfo *command_info,
                           void *message_data) {
  LOG_INFO<<"run SendMessage!";
  return SendCMD(command_info->cmd_set, command_info->cmd_id,
                 command_info->receiver, message_data, command_info->length,
                 CMDSessionMode::CMD_SESSION_0);
}

/*************************** Session Management **************************/
void Protocol::SetupSession() {
  uint8_t i, j;
  for (i = 0; i < SESSION_TABLE_NUM; i++) {
    cmd_session_table_[i].session_id = i;
    cmd_session_table_[i].usage_flag = false;
    cmd_session_table_[i].memory_block_ptr = nullptr;
  }

  for (i = 0; i < RECEIVER_NUM; i++) {
    for (j = 0; j < (SESSION_TABLE_NUM - 1); j++) {
      ack_session_table_[i][j].session_id = j + 1;
      ack_session_table_[i][j].session_status = ACKSessionStatus::ACK_SESSION_IDLE;
      ack_session_table_[i][j].memory_block_ptr = nullptr;
    }
  }
}

CMDSession *Protocol::AllocCMDSession(CMDSessionMode session_mode, uint16_t size) {
  uint32_t i;
  MemoryBlock *memory_block_ptr = nullptr;

  if (session_mode == CMDSessionMode::CMD_SESSION_0 || session_mode == CMDSessionMode::CMD_SESSION_1) {
    if (cmd_session_table_[(uint16_t) session_mode].usage_flag == 0) {
      i = static_cast<uint32_t>(session_mode);
    } else {
      DLOG_ERROR << "session " << static_cast<uint32_t>(session_mode) << " is busy\n";
      return nullptr;
    }
  } else {
    for (i = 2; i < SESSION_TABLE_NUM; i++) {
      if (cmd_session_table_[i].usage_flag == 0) {
        break;
      }
    }

  }

  if (i < 32 && cmd_session_table_[i].usage_flag == 0) {

    cmd_session_table_[i].usage_flag = 1;
    memory_block_ptr = memory_pool_ptr_->AllocMemory(size);
    if (memory_block_ptr == nullptr) {
      cmd_session_table_[i].usage_flag = 0;
    } else {
//      DLOG_INFO<<"find "<<i;
      cmd_session_table_[i].memory_block_ptr = memory_block_ptr;
      return &cmd_session_table_[i];
    }
  } else {
    DLOG_INFO << "All usable CMD session id are occupied";
  }

  return nullptr;
}

void Protocol::FreeCMDSession(CMDSession *session_ptr) {
  if (session_ptr->usage_flag == 1) {
    memory_pool_ptr_->FreeMemory(session_ptr->memory_block_ptr);
    session_ptr->usage_flag = 0;
  }
}

ACKSession *Protocol::AllocACKSession(uint8_t receiver, uint16_t session_id, uint16_t size) {
  MemoryBlock *memory_block_ptr = nullptr;
  if (session_id > 0 && session_id < 32) {
    if (ack_session_table_[receiver][session_id - 1].memory_block_ptr) {
      FreeACKSession(&ack_session_table_[receiver][session_id - 1]);
    }

    memory_block_ptr = memory_pool_ptr_->AllocMemory(size);
    if (memory_block_ptr == nullptr) {
      DLOG_ERROR << "there is not enough memory";
      return nullptr;
    } else {
      ack_session_table_[receiver][session_id - 1].memory_block_ptr = memory_block_ptr;
      return &ack_session_table_[receiver][session_id - 1];
    }
  }
  return nullptr;
}

void Protocol::FreeACKSession(ACKSession *session_ptr) {
  memory_pool_ptr_->FreeMemory(session_ptr->memory_block_ptr);
}

/****************************** Send Pipline *****************************/
bool Protocol::SendCMD(uint8_t cmd_set, uint8_t cmd_id, uint8_t receiver,
                       void *data_ptr, uint16_t data_length,
                       CMDSessionMode session_mode, MessageHeader* message_header,
                       std::chrono::milliseconds ack_timeout, int retry_time) {

  CMDSession *cmd_session_ptr = nullptr;
  Header *header_ptr = nullptr;
  uint8_t cmd_set_prefix[] = {cmd_id, cmd_set};
 //LX CHANGE 0325
 // uint32_t crc_data;
 uint16_t crc_data;
 //END 0325
  uint16_t pack_length = 0;


  //calculate pack_length first
  if (data_length == 0 || data_ptr == nullptr) {
    DLOG_ERROR << "No data send.";
    return false;
  }
  pack_length = HEADER_LEN +
      CMD_SET_PREFIX_LEN +
      data_length + CRC_DATA_LEN;

  //second get the param into the session
      //lock
      memory_pool_ptr_->LockMemory();
      cmd_session_ptr = AllocCMDSession(CMDSessionMode::CMD_SESSION_0, pack_length);

      if (cmd_session_ptr == nullptr) {
        //unlock
        memory_pool_ptr_->UnlockMemory();
        DLOG_ERROR << "Allocate CMD session failed.";
        return false;
      }

      //pack into cmd_session memory_block
      header_ptr = (Header *) cmd_session_ptr->memory_block_ptr->memory_ptr;
      header_ptr->sof = SOF;
      header_ptr->length = pack_length - HEADER_LEN - CRC_DATA_LEN;
      LOG_INFO<< " header_ptr->length  "<<      header_ptr->length ;
      header_ptr->version = VERSION;
      header_ptr->session_id = cmd_id;
      //header_ptr->is_ack = 0;
      //header_ptr->reserved0 = 0;
      header_ptr->sender = DEVICE;
      header_ptr->receiver = receiver;
      //LOG_INFO<<header_ptr->sender<<header_ptr->receiver;
      //header_ptr->reserved1 = 0;
     // header_ptr->seq_num = seq_num_;
      //header_ptr->crc = CRC16Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, HEADER_LEN - CRC_HEAD_LEN);

      if(message_header){
       // message_header->is_ack = false;
       // message_header->seq_num = seq_num_;
        message_header->session_id = cmd_session_ptr->session_id;
      }

      // pack the cmd prefix ,data and data crc into memory block one by one
      //memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN, cmd_set_prefix, CMD_SET_PREFIX_LEN);
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN + CMD_SET_PREFIX_LEN, data_ptr, data_length);

      crc_data = CRC32Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, pack_length - CRC_DATA_LEN);
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + pack_length - CRC_DATA_LEN, &crc_data, CRC_DATA_LEN);

      // send it using device
      DeviceSend(cmd_session_ptr->memory_block_ptr->memory_ptr);

      seq_num_++;
      FreeCMDSession(cmd_session_ptr);
      //unlock
      memory_pool_ptr_->UnlockMemory();


  return true;

}

bool Protocol::SendACK(uint8_t session_id, uint16_t seq_num, uint8_t receiver,
                       void *ack_ptr, uint16_t ack_length) {
  ACKSession *ack_session_ptr = nullptr;
  Header *header_ptr = nullptr;
  // LX CHANGE 0325
  // uint32_t crc_data = 0;
  uint16_t crc_data = 0;
  //END 0325
  uint16_t pack_length = 0;

  if (ack_ptr == nullptr || ack_length == 0) {
    pack_length = HEADER_LEN;
  } else {
    pack_length = HEADER_LEN +
        ack_length + CRC_DATA_LEN;
  }

  if (session_id == 0 || session_id > 31) {
    DLOG_ERROR << ("ack session id should be from 1 to 31.");
    return false;
  } else {

    //lock
    memory_pool_ptr_->LockMemory();
    ack_session_ptr = AllocACKSession(receiver, session_id, pack_length);

    if (ack_session_ptr == nullptr) {
      //unlock
      memory_pool_ptr_->UnlockMemory();
      DLOG_ERROR << "Allocate ACK session failed.";
      return false;
    }

    //pack into ack_session memory_block
    header_ptr = (Header *) ack_session_ptr->memory_block_ptr->memory_ptr;
    header_ptr->sof = SOF;
    header_ptr->length = pack_length - HEADER_LEN - CRC_DATA_LEN;
    header_ptr->version = VERSION;
    header_ptr->session_id = ack_session_ptr->session_id;
    //header_ptr->is_ack = 1;
    //header_ptr->reserved0 = 0;
    header_ptr->sender = DEVICE;
    header_ptr->receiver = receiver;
   // LOG_INFO<<header_ptr->sender<<header_ptr->receiver;
    //header_ptr->reserved1 = 0;
    //header_ptr->seq_num = seq_num;
    //header_ptr->crc = CRC16Calc(ack_session_ptr->memory_block_ptr->memory_ptr, HEADER_LEN - CRC_HEAD_LEN);

    // pack the cmd prefix ,data and data crc into memory block one by one
    if (ack_ptr != nullptr && ack_length != 0) {
      memcpy(ack_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN, ack_ptr, ack_length);
      crc_data = CRC32Calc(ack_session_ptr->memory_block_ptr->memory_ptr, pack_length - CRC_DATA_LEN);
      memcpy(ack_session_ptr->memory_block_ptr->memory_ptr + pack_length - CRC_DATA_LEN, &crc_data, CRC_DATA_LEN);
    }

    // send it using device
    DeviceSend(ack_session_ptr->memory_block_ptr->memory_ptr);
    ack_session_table_[receiver][session_id - 1].session_status = ACKSessionStatus::ACK_SESSION_USING;
    //unlock
    memory_pool_ptr_->UnlockMemory();
    return true;
  }

}

bool Protocol::DeviceSend(uint8_t *buf) {
  int ans;
  Header *header_ptr = (Header *) buf;

// For debug and visualzation:
// ans = header_ptr->length;
//  for(int i =0;i<header_ptr->length;i++){
//    printf("send_byte %d:\t %X\n ", i, buf[i]);
//  }
//  std::cout<<"----------------"<<std::endl;
  ans = serial_device_ptr_->Write(buf, header_ptr->length + (HEADER_LEN +CRC_DATA_LEN));

  if (ans <= 0) {
    DLOG_ERROR << "Port failed.";
  } else if (ans != header_ptr->length + (HEADER_LEN +CRC_DATA_LEN)) {
    DLOG_ERROR << "Port send failed, send length:" << ans << "package length" << header_ptr->length + (HEADER_LEN +CRC_DATA_LEN);
  } else {
    DLOG_INFO << "Port send success.";
    return true;
  }
  return false;
}

/****************************** Recv Pipline ******************************/
RecvContainer *Protocol::Receive() {

  //! Bool to check if the protocol parser has finished a full frame
  bool is_frame = false;
      //LOG_INFO<<"run Receive 1: "<<recv_buff_read_pos_<<"   "<<   recv_buff_read_len_;
  //! Step 1: Check if the buffer has been consumed
  if (recv_buff_read_pos_ >= recv_buff_read_len_) {
   // LOG_INFO<<"run buffer has been consumed ";
    recv_buff_read_pos_ = 0;
    recv_buff_read_len_ = serial_device_ptr_->Read(recv_buff_ptr_, BUFFER_SIZE);
  }
 // LOG_INFO<<"run Receive step 2";
  //! Step 2:
  //! For large data protocol, store the value and only verify the header
  //! For small data protocol, Go through the buffer and return when you
  //! see a full frame. buf_read_pos will maintain state about how much
  //! buffer data we have already read

  //TODO: unhandled
  if (is_large_data_protocol_ && recv_buff_read_len_ == BUFFER_SIZE) {
      LOG_INFO<<"run large data  ";
    memcpy(recv_stream_ptr_->recv_buff + (recv_stream_ptr_->recv_index), recv_buff_ptr_,
           BUFFER_SIZE);
    recv_stream_ptr_->recv_index += BUFFER_SIZE;
    recv_buff_read_pos_ = BUFFER_SIZE;
  } else {
      //LOG_INFO<<"run buffer not consumed ";

    for (recv_buff_read_pos_; recv_buff_read_pos_ < recv_buff_read_len_;
         recv_buff_read_pos_++) {
        //LOG_INFO<<"run Receive 2: "<<recv_buff_read_pos_<<"   "<<   recv_buff_read_len_;
      is_frame = ByteHandler(recv_buff_ptr_[recv_buff_read_pos_]);

      if (is_frame) {
        return recv_container_ptr_;
      }
    }
  }

  //! Step 3: If we don't find a full frame by this time, return nullptr.
  return nullptr;
}

bool Protocol::ByteHandler(const uint8_t byte) {
  recv_stream_ptr_->reuse_count = 0;
  recv_stream_ptr_->reuse_index = MAX_PACK_SIZE;

  bool is_frame = StreamHandler(byte);
//LOG_INFO<<"run ByteHandler";
  if (reuse_buffer_) {
   // LOG_INFO<<"run reuse_buffer_ is ture";
    if (recv_stream_ptr_->reuse_count != 0) {
      while (recv_stream_ptr_->reuse_index < MAX_PACK_SIZE) {
        /*! @note because reuse_index maybe re-located, so reuse_index must
         *  be
         *  always point to un-used index
         *  re-loop the buffered data
         *  */
        is_frame = StreamHandler(recv_stream_ptr_->recv_buff[recv_stream_ptr_->reuse_index++]);
      }
      recv_stream_ptr_->reuse_count = 0;
    }
  }
  return is_frame;
}

bool Protocol::StreamHandler(uint8_t byte) {
  //LOG_INFO<<"run StreamHandler";
  // push the byte into filter buffer
  if (recv_stream_ptr_->recv_index < MAX_PACK_SIZE) {
    recv_stream_ptr_->recv_buff[recv_stream_ptr_->recv_index] = byte;
    recv_stream_ptr_->recv_index++;
    //LOG_INFO<<"  recv_index1:"<<recv_stream_ptr_->recv_index;
  } else {
    LOG_ERROR << "Buffer overflow";
    memset(recv_stream_ptr_->recv_buff, 0, recv_stream_ptr_->recv_index);
    recv_stream_ptr_->recv_index = 0;
  }

  bool is_frame = CheckStream();
  return is_frame;
}

bool Protocol::CheckStream() {
  Header *header_ptr = (Header *) (recv_stream_ptr_->recv_buff);
  //header_ptr->length+=(HEADER_LEN + CRC_DATA_LEN);
 // LOG_INFO<<"header_ptr->length ="<<header_ptr->length+(HEADER_LEN + CRC_DATA_LEN);
  bool is_frame = false;
  if (recv_stream_ptr_->recv_index < HEADER_LEN) {
    return false;
  } else if (recv_stream_ptr_->recv_index == HEADER_LEN) {
    is_frame = VerifyHeader();
   // LOG_INFO<<"head check is ok!  is_frame:"<<is_frame;
  } else if (recv_stream_ptr_->recv_index == header_ptr->length +(HEADER_LEN + CRC_DATA_LEN) ) {
    //LOG_INFO<<"run VerifyData";
    is_frame = VerifyData();
  }

  return is_frame;
}

bool Protocol::VerifyHeader() {
  Header *header_ptr = (Header *) (recv_stream_ptr_->recv_buff);
  //header_ptr->length+=(HEADER_LEN + CRC_DATA_LEN);
  bool is_frame = false;
 //LOG_INFO<<"SOF:"<<header_ptr->sof<<"~"<<SOF<<"  VERSION:"<<header_ptr->version<<"~"<<VERSION\
 <<"  length:"<< header_ptr->length+(HEADER_LEN + CRC_DATA_LEN)<<"  receiver:"<<header_ptr->receiver<<"~"<<DEVICE\
 <<"  HEADER_LEN:"<<HEADER_LEN;
  if ((header_ptr->sof == SOF) && (header_ptr->version == VERSION) &&
      (header_ptr->length+(HEADER_LEN + CRC_DATA_LEN) < MAX_PACK_SIZE) &&
      (header_ptr->receiver == DEVICE || header_ptr->receiver == 0xFF)) {
     // LOG_INFO<<"VerifyHeader check is ok !!!!";
    // It is an unused part because minimum package is more longer than a header
    //LX CHANGE 0322
    // maybe length > HEADER_LEN
    if (header_ptr->length+(HEADER_LEN + CRC_DATA_LEN) == HEADER_LEN) {
    //END 0322
      is_frame = ContainerHandler();
      //prepare data stream
      PrepareStream();
    }
  } else {
    //shift the data stream
    ShiftStream();
  }

  return is_frame;

}

bool Protocol::VerifyData() {
  Header *header_ptr = (Header *) (recv_stream_ptr_->recv_buff);
  //header_ptr->length+=(HEADER_LEN + CRC_DATA_LEN);
  bool is_frame = false;
  if (CRCTailCheck((uint8_t *) header_ptr, header_ptr->length+(HEADER_LEN + CRC_DATA_LEN))) {

    is_frame = ContainerHandler();
    //prepare data stream
    PrepareStream();
  } else {
    //reuse the data stream
    ReuseStream();
  }

  return is_frame;
}

bool Protocol::ContainerHandler() {

  Header *session_header_ptr = nullptr;
  Header *header_ptr = (Header *) (recv_stream_ptr_->recv_buff);
  //header_ptr->length+=(HEADER_LEN + CRC_DATA_LEN);
  bool is_frame = false;

        //recv_container_ptr_->message_header.is_ack = false;
        //recv_container_ptr_->message_header.seq_num = header_ptr->seq_num;
        recv_container_ptr_->message_header.session_id = header_ptr->session_id;
        recv_container_ptr_->command_info.sender = header_ptr->sender;
        recv_container_ptr_->command_info.receiver = header_ptr->receiver;
        recv_container_ptr_->command_info.length = header_ptr->length+(HEADER_LEN + CRC_DATA_LEN) - HEADER_LEN - CMD_SET_PREFIX_LEN - CRC_DATA_LEN;
        //LX CHANGE 0325
        //recv_container_ptr_->command_info.cmd_set = *((uint8_t *) header_ptr + HEADER_LEN + 1);
        //recv_container_ptr_->command_info.cmd_id = *((uint8_t *) header_ptr + HEADER_LEN);
        recv_container_ptr_->command_info.cmd_set = CMD_SET_CONST;
        recv_container_ptr_->command_info.cmd_id =header_ptr->session_id;
        //recv_container_ptr_->command_info.need_ack = false;
        //END 0325
        // for(int i=0;i<header_ptr->length+(HEADER_LEN + CRC_DATA_LEN);i++)
        //{
        //     LOG_INFO<<" | "<<  (int)*((uint8_t *) header_ptr + i)<<" | ";
        //}

        memcpy(recv_container_ptr_->message_data.raw_data, (uint8_t *) header_ptr + HEADER_LEN ,
               header_ptr->length+(HEADER_LEN + CRC_DATA_LEN) - HEADER_LEN - CRC_DATA_LEN);

        is_frame = true;
//        LOG_INFO<<"run memcpy:"<<header_ptr->length+(HEADER_LEN + CRC_DATA_LEN) - HEADER_LEN - CMD_SET_PREFIX_LEN - CRC_DATA_LEN<<" HEADER_LEN:"<<HEADER_LEN;

  return is_frame;
}

/*************************** Stream Management*****************************/
void Protocol::PrepareStream() {
  uint32_t bytes_to_move = HEADER_LEN - 1;
  uint32_t index_of_move = recv_stream_ptr_->recv_index - bytes_to_move;

  memmove(recv_stream_ptr_->recv_buff, recv_stream_ptr_->recv_buff + index_of_move, bytes_to_move);
  memset(recv_stream_ptr_->recv_buff + bytes_to_move, 0, index_of_move);
  recv_stream_ptr_->recv_index = bytes_to_move;
 // LOG_INFO<<"RUN PrepareStream  recv_index:"<< recv_stream_ptr_->recv_index;
}

void Protocol::ShiftStream() {
  if (recv_stream_ptr_->recv_index) {
     // LOG_INFO<<"RUN ShiftStream  recv_index:"<< recv_stream_ptr_->recv_index;
    recv_stream_ptr_->recv_index--;
    if (recv_stream_ptr_->recv_index) {
      memmove(recv_stream_ptr_->recv_buff, recv_stream_ptr_->recv_buff + 1, recv_stream_ptr_->recv_index);
    }
  }
}

void Protocol::ReuseStream() {
  uint8_t *buff_ptr = recv_stream_ptr_->recv_buff;
  uint16_t bytes_to_move = recv_stream_ptr_->recv_index - HEADER_LEN;
  uint8_t *src_ptr = buff_ptr + HEADER_LEN;

  uint16_t n_dest_index = recv_stream_ptr_->reuse_index - bytes_to_move;
  uint8_t *dest_ptr = buff_ptr + n_dest_index;

  memmove(dest_ptr, src_ptr, bytes_to_move);

  recv_stream_ptr_->recv_index = HEADER_LEN;
  ShiftStream();

  recv_stream_ptr_->recv_index = n_dest_index;
  recv_stream_ptr_->reuse_count++;
}

/*************************** CRC Calculationns ****************************/
uint16_t Protocol::CRC16Update(uint16_t crc, uint8_t ch) {
  uint16_t tmp;
  uint16_t msg;

  msg = 0x00ff & static_cast<uint16_t>(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

  return crc;
}

uint32_t Protocol::CRC32Update(uint32_t crc, uint8_t ch) {
  uint32_t tmp;
  uint32_t msg;

  msg = 0x000000ffL & static_cast<uint32_t>(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab32[tmp & 0xff];
  return crc;
}

uint16_t Protocol::CRC16Calc(const uint8_t *data_ptr, size_t length) {
  size_t i;
  uint16_t crc = CRC_INIT;

  for (i = 0; i < length; i++) {
    crc = CRC16Update(crc, data_ptr[i]);
  }

  return crc;
}

//LX CHANGE 0325
/*orignal
uint32_t Protocol::CRC32Calc(const uint8_t *data_ptr, size_t length) {
  size_t i;
  uint32_t crc = CRC_INIT;

  for (i = 0; i < length; i++) {
    crc = CRC32Update(crc, data_ptr[i]);
  }

  return crc;
}
 */
//!set Frame tail always is  CRC_INIT
 uint16_t Protocol::CRC32Calc(const uint8_t *data_ptr, size_t length) {
  uint16_t crc = CRC_INIT;
  return crc;
}
//END
//LX CHANGE 0322
//Change crc check always ok!
/*
bool Protocol::CRCHeadCheck(uint8_t *data_ptr, size_t length) {
  if (CRC16Calc(data_ptr, length) == 0) {
    return true;
  } else {
    return false;
  }
}
 */
    bool Protocol::CRCHeadCheck(uint8_t *data_ptr, size_t length) {
        return true;
    }
/*
bool Protocol::CRCTailCheck(uint8_t *data_ptr, size_t length) {
  if (CRC32Calc(data_ptr, length) == 0) {
    return true;
  } else {
    return false;
  }
}
*/
//! tail crc check
    bool Protocol::CRCTailCheck(uint8_t *data_ptr, size_t length) {
      if((data_ptr[length-1]==HALF_CRC_INIT)&&(data_ptr[length-2]==HALF_CRC_INIT))
        return true;
      else
        return false;
    }
//END 0322
}

