//#include <regex>
#include <mutex>

#include "Crazyflie.h"
#include "crtp.h"

#include "Crazyradio.h"

#include <iostream>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "num.h"

#define MAX_RADIOS 16

Crazyradio* g_crazyradios[MAX_RADIOS];
std::mutex g_mutex[MAX_RADIOS];

Crazyflie::Crazyflie(
  const std::string& link_uri)
  : m_radio(NULL)
  , m_devId(0)
  , m_channel(0)
  , m_address(0)
  , m_datarate(Crazyradio::Datarate_250KPS)
  , m_logInfo()
  , m_logTocEntries()
  , m_logTocEntriesRequested()
  , m_logBlockCb()
  , m_blockReset(false)
  , m_blockCreated()
  , m_blockStarted()
  , m_blockStopped()
  , m_paramInfo()
  , m_paramTocEntries()
  , m_paramTocEntriesRequested()
  , m_paramValues()
  , m_paramValuesRequested()
  , m_emptyAckCallback(nullptr)
  , m_linkQualityCallback(nullptr)
  , m_lastTrajectoryId(0)
  , m_lastTrajectoryResponse(-1)
  , m_lastTrajectoryResponse2(-1)
  , m_lastTrajectoryResponse3(-1)
{
  int datarate;
  int channel;
  char datarateType;
  bool success = false;

  success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%lx",
     &m_devId, &channel, &datarate,
     &datarateType, &m_address) == 5;
  if (!success) {
    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c",
       &m_devId, &channel, &datarate,
       &datarateType) == 4;
    m_address = 0xE7E7E7E7E7;
  }

  if (success)
  {
    m_channel = channel;
    if (datarate == 250 && datarateType == 'K') {
      m_datarate = Crazyradio::Datarate_250KPS;
    }
    else if (datarate == 1 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_1MPS;
    }
    else if (datarate == 2 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_2MPS;
    }

    if (m_devId >= MAX_RADIOS) {
      throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
    }

    {
      std::unique_lock<std::mutex> mlock(g_mutex[m_devId]);
      if (!g_crazyradios[m_devId]) {
        g_crazyradios[m_devId] = new Crazyradio(m_devId);
        // g_crazyradios[m_devId]->setAckEnable(false);
        g_crazyradios[m_devId]->setAckEnable(true);
        g_crazyradios[m_devId]->setArc(0);
      }
    }

    m_radio = g_crazyradios[m_devId];
  }
  else {
    throw std::runtime_error("Uri is not valid!");
  }
}

void Crazyflie::logReset()
{
  m_blockReset = false;
  do {
    crtpLogResetRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } while (!m_blockReset);
}

// void Crazyflie::sendSetpoint(
//   float roll,
//   float pitch,
//   float yawrate,
//   uint16_t thrust)
// {
//   crtpSetpointRequest request(roll, pitch, yawrate, thrust);
//   sendPacket((const uint8_t*)&request, sizeof(request));
// }

void Crazyflie::sendPing()
{
  uint8_t ping = 0xFF;
  sendPacket(&ping, sizeof(ping));
}

// https://forum.bitcraze.io/viewtopic.php?f=9&t=1488
void Crazyflie::reboot()
{
  const uint8_t reboot_init[] = {0xFF, 0xFE, 0xFF};
  while(!sendPacket(reboot_init, sizeof(reboot_init))) {}

  const uint8_t reboot_to_firmware[] = {0xFF, 0xFE, 0xF0, 0x01};
  while(!sendPacket(reboot_to_firmware, sizeof(reboot_to_firmware))) {}
}

void Crazyflie::rebootToBootloader()
{
  const uint8_t reboot_init[] = {0xFF, 0xFE, 0xFF};
  while(!sendPacket(reboot_init, sizeof(reboot_init))) {}

  const uint8_t reboot_to_bootloader[] = {0xFF, 0xFE, 0xF0, 0x00};
  while(!sendPacket(reboot_to_bootloader, sizeof(reboot_to_bootloader))) {}
}

void Crazyflie::requestLogToc()
{

  // Find the number of log variables in TOC
  m_logInfo.len = 0;
  do
  {
    crtpLogGetInfoRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
  } while(m_logInfo.len == 0);
  std::cout << "Log: " << (int)m_logInfo.len << std::endl;

  // Prepare data structures to request detailed information
  m_logTocEntriesRequested.clear();
  m_logTocEntries.resize(m_logInfo.len);
  for (size_t i = 0; i < m_logInfo.len; ++i)
  {
    m_logTocEntriesRequested.insert(i);
  }

  // Request detailed information, until done
  while (m_logTocEntriesRequested.size() > 0)
  {
    for (size_t p = 0; p < m_logTocEntriesRequested.size(); ++p)
    {
      auto iter = m_logTocEntriesRequested.begin();
      for (size_t j = 0; j < p; ++j) {
        ++iter;
      }
      size_t i = *iter;
      crtpLogGetItemRequest request(i);
      sendPacket((const uint8_t*)&request, sizeof(request));
    }
  }
}

void Crazyflie::requestParamToc()
{
  // Find the number of log variables in TOC
  m_paramInfo.len = 0;
  do
  {
    crtpParamTocGetInfoRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
  } while(m_paramInfo.len == 0);
  std::cout << "Params: " << (int)m_paramInfo.len << std::endl;

  // Prepare data structures to request detailed information
  m_paramTocEntriesRequested.clear();
  m_paramValues.clear();
  m_paramTocEntries.resize(m_paramInfo.len);
  for (size_t i = 0; i < m_paramInfo.len; ++i)
  {
    m_paramTocEntriesRequested.insert(i);
    m_paramValuesRequested.insert(i);
  }

  // Request detailed information, until done
  while (m_paramTocEntriesRequested.size() > 0)
  {
    for (size_t p = 0; p < m_paramTocEntriesRequested.size(); ++p)
    {
      auto iter = m_paramTocEntriesRequested.begin();
      for (size_t j = 0; j < p; ++j) {
        ++iter;
      }
      size_t i = *iter;
      crtpParamTocGetItemRequest request(i);
      sendPacket((const uint8_t*)&request, sizeof(request));
    }
  }

  // Request current values
  while (m_paramValuesRequested.size() > 0)
  {
    for (size_t p = 0; p < m_paramValuesRequested.size(); ++p)
    {
      auto iter = m_paramValuesRequested.begin();
      for (size_t j = 0; j < p; ++j) {
        ++iter;
      }
      size_t i = *iter;
      crtpParamReadRequest request(i);
      sendPacket((const uint8_t*)&request, sizeof(request));
    }
  }
}

void Crazyflie::setParam(uint8_t id, const ParamValue& value) {

  m_paramValues.erase(id);
  for (auto&& entry : m_paramTocEntries) {
    if (entry.id == id) {
      do
      {
        switch (entry.type) {
          case ParamTypeUint8:
            {
              crtpParamWriteRequest<uint8_t> request(id, value.valueUint8);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeInt8:
            {
              crtpParamWriteRequest<int8_t> request(id, value.valueInt8);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeUint16:
            {
              crtpParamWriteRequest<uint16_t> request(id, value.valueUint16);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeInt16:
            {
              crtpParamWriteRequest<int16_t> request(id, value.valueInt16);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeUint32:
            {
              crtpParamWriteRequest<uint32_t> request(id, value.valueUint32);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeInt32:
            {
              crtpParamWriteRequest<int32_t> request(id, value.valueInt32);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeFloat:
            {
              crtpParamWriteRequest<float> request(id, value.valueFloat);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
        }
      } while(m_paramValues.find(id) == m_paramValues.end());
      break;
    }
  }


}

void Crazyflie::trajectoryReset()
{
  m_lastTrajectoryResponse = -1;
  do {
    crtpTrajectoryResetRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } while (m_lastTrajectoryResponse != 0);
  m_lastTrajectoryId = 0;
}

void Crazyflie::trajectoryAdd(
    float duration,
    std::vector<float> poly_x,
    std::vector<float> poly_y,
    std::vector<float> poly_z,
    std::vector<float> poly_yaw)
{
  crtpTrajectoryAddRequest request;
  request.id = m_lastTrajectoryId;

  std::cout << duration << std::endl;
  for (size_t i = 0; i < poly_x.size(); ++i) {
    std::cout << poly_x[i] << ",";
  }
  std::cout << std::endl;

  // Part 1
  request.offset = 0;
  request.size = 6;
  request.values[0] = duration;
  request.values[1] = poly_x[0];
  request.values[2] = poly_x[1];
  request.values[3] = poly_x[2];
  request.values[4] = poly_x[3];
  request.values[5] = poly_x[4];

  m_lastTrajectoryResponse = -1;
  do {
    sendPacket((const uint8_t*)&request, sizeof(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (m_lastTrajectoryResponse != 1 || m_lastTrajectoryResponse2 != m_lastTrajectoryId
          || m_lastTrajectoryResponse3 != ((const uint8_t*)&request)[3]);

  // Part 2
  request.offset = 6;
  request.size = 6;
  request.values[0] = poly_x[5];
  request.values[1] = poly_x[6];
  request.values[2] = poly_x[7];
  request.values[3] = poly_y[0];
  request.values[4] = poly_y[1];
  request.values[5] = poly_y[2];

  m_lastTrajectoryResponse = -1;
  do {
    sendPacket((const uint8_t*)&request, sizeof(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (m_lastTrajectoryResponse != 1 || m_lastTrajectoryResponse2 != m_lastTrajectoryId
          || m_lastTrajectoryResponse3 != ((const uint8_t*)&request)[3]);

  // Part 3
  request.offset = 12;
  request.size = 6;
  request.values[0] = poly_y[3];
  request.values[1] = poly_y[4];
  request.values[2] = poly_y[5];
  request.values[3] = poly_y[6];
  request.values[4] = poly_y[7];
  request.values[5] = poly_z[0];

  m_lastTrajectoryResponse = -1;
  do {
    sendPacket((const uint8_t*)&request, sizeof(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (m_lastTrajectoryResponse != 1 || m_lastTrajectoryResponse2 != m_lastTrajectoryId
          || m_lastTrajectoryResponse3 != ((const uint8_t*)&request)[3]);

  // Part 4
  request.offset = 18;
  request.size = 6;
  request.values[0] = poly_z[1];
  request.values[1] = poly_z[2];
  request.values[2] = poly_z[3];
  request.values[3] = poly_z[4];
  request.values[4] = poly_z[5];
  request.values[5] = poly_z[6];

  m_lastTrajectoryResponse = -1;
  do {
    sendPacket((const uint8_t*)&request, sizeof(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (m_lastTrajectoryResponse != 1 || m_lastTrajectoryResponse2 != m_lastTrajectoryId
          || m_lastTrajectoryResponse3 != ((const uint8_t*)&request)[3]);

  // Part 5
  request.offset = 24;
  request.size = 6;
  request.values[0] = poly_z[7];
  request.values[1] = poly_yaw[0];
  request.values[2] = poly_yaw[1];
  request.values[3] = poly_yaw[2];
  request.values[4] = poly_yaw[3];
  request.values[5] = poly_yaw[4];

  m_lastTrajectoryResponse = -1;
  do {
    sendPacket((const uint8_t*)&request, sizeof(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (m_lastTrajectoryResponse != 1 || m_lastTrajectoryResponse2 != m_lastTrajectoryId
          || m_lastTrajectoryResponse3 != ((const uint8_t*)&request)[3]);

  // Part 6
  request.offset = 30;
  request.size = 3;
  request.values[0] = poly_yaw[5];
  request.values[1] = poly_yaw[6];
  request.values[2] = poly_yaw[7];

  m_lastTrajectoryResponse = -1;
  do {
    sendPacket((const uint8_t*)&request, sizeof(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (m_lastTrajectoryResponse != 1 || m_lastTrajectoryResponse2 != m_lastTrajectoryId
          || m_lastTrajectoryResponse3 != ((const uint8_t*)&request)[3]);

  ++m_lastTrajectoryId;
}

void Crazyflie::trajectoryHover(
    float x,
    float y,
    float z,
    float yaw)
{
  crtpTrajectoryHoverRequest request(x, y, z, yaw);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

// void Crazyflie::trajectoryStart()
// {
//   m_lastTrajectoryResponse = -1;
//   do {
//     crtpTrajectoryStartRequest request;
//     sendPacket((const uint8_t*)&request, sizeof(request));
//     std::this_thread::sleep_for(std::chrono::milliseconds(50));
//   } while (m_lastTrajectoryResponse != 2);
// }

// void Crazyflie::setTrajectoryState(bool state)
// {
//   m_lastTrajectoryResponse = -1;
//   do {
//     crtpTrajectoryStateRequest request(state);
//     sendPacket((const uint8_t*)&request, sizeof(request));
//     std::this_thread::sleep_for(std::chrono::milliseconds(50));
//   } while (m_lastTrajectoryResponse != 3 || m_lastTrajectoryResponse2 != state);
// }

void Crazyflie::sendPositionExternalBringup(
  const stateExternalBringup& data)
{
  crtpPosExtBringup request;
  request.pose[0].id = data.id;
  request.pose[0].x = single2half(data.x);
  request.pose[0].y = single2half(data.y);
  request.pose[0].z = single2half(data.z);
  request.pose[0].quat[0] = int16_t(data.q0 * 32768.0);
  request.pose[0].quat[1] = int16_t(data.q1 * 32768.0);
  request.pose[0].quat[2] = int16_t(data.q2 * 32768.0);
  request.pose[0].quat[3] = int16_t(data.q3 * 32768.0);
  request.pose[1].id = 0;
  sendPacket((const uint8_t*)&request, sizeof(request));
}


bool Crazyflie::sendPacket(
  const uint8_t* data,
  uint32_t length)
{
  static uint32_t numPackets = 0;
  static uint32_t numAcks = 0;

  numPackets++;

  Crazyradio::Ack ack;
  {
    std::unique_lock<std::mutex> mlock(g_mutex[m_devId]);
    if (m_radio->getAddress() != m_address) {
      m_radio->setAddress(m_address);
    }
    if (m_radio->getChannel() != m_channel) {
      m_radio->setChannel(m_channel);
    }
    if (m_radio->getDatarate() != m_datarate) {
      m_radio->setDatarate(m_datarate);
    }
    if (!m_radio->getAckEnable()) {
      m_radio->setAckEnable(true);
    }
    m_radio->sendPacket(data, length, ack);
  }
  ack.data[ack.size] = 0;
  if (ack.ack) {
    handleAck(ack);
    numAcks++;
  }
  if (numPackets == 100) {
    if (m_linkQualityCallback) {
      // We just take the ratio of sent vs. acked packets here
      // for a sliding window of 100 packets
      float linkQuality = numAcks / (float)numPackets;
      m_linkQualityCallback(linkQuality);
    }
    numPackets = 0;
    numAcks = 0;
  }
  return ack.ack;
}

void Crazyflie::handleAck(
  const Crazyradio::Ack& result)
{
  if (crtpConsoleResponse::match(result)) {
    if (result.size > 0) {
      crtpConsoleResponse* r = (crtpConsoleResponse*)result.data;
      std::cout << r->text << std::endl;
    }
    // ROS_INFO("Console: %s", r->text);
  }
  else if (crtpLogGetInfoResponse::match(result)) {
    crtpLogGetInfoResponse* r = (crtpLogGetInfoResponse*)result.data;
    m_logInfo.len = r->log_len;
  }
  else if (crtpLogGetItemResponse::match(result)) {
    crtpLogGetItemResponse* r = (crtpLogGetItemResponse*)result.data;
    if (r->request.id < m_logTocEntries.size())
    {
      m_logTocEntriesRequested.erase(r->request.id);
      LogTocEntry& entry = m_logTocEntries[r->request.id];
      entry.id = r->request.id;
      entry.type = (LogType)r->type;
      entry.group = std::string(&r->text[0]);
      entry.name = std::string(&r->text[entry.group.size() + 1]);
    }
    // std::cout << "Got: " << (int)r->id << std::endl;
  }
  else if (crtpLogControlResponse::match(result)) {
    crtpLogControlResponse* r = (crtpLogControlResponse*)result.data;
    if (r->command == 0 && r->result == 0) {
      m_blockCreated.insert(r->requestByte1);
    }
    if (r->command == 3 && r->result == 0) {
      m_blockStarted.insert(r->requestByte1);
    }
    if (r->command == 4 && r->result == 0) {
      m_blockStopped.insert(r->requestByte1);
    }
    if (r->command == 5 && r->result == 0) {
      m_blockReset = true;
    }
    // std::cout << "LogControl: " << (int)r->command << " errno: " << (int)r->result << std::endl;
  }
  else if (crtpLogDataResponse::match(result)) {
    crtpLogDataResponse* r = (crtpLogDataResponse*)result.data;
    auto iter = m_logBlockCb.find(r->blockId);
    if (iter != m_logBlockCb.end()) {
      iter->second(r, result.size - 5);
    }
    else {
      std::cout << "Received unrequested data for block: " << (int)r->blockId << std::endl;
    }
  }
  else if (crtpParamTocGetInfoResponse::match(result)) {
    crtpParamTocGetInfoResponse* r = (crtpParamTocGetInfoResponse*)result.data;
    m_paramInfo.len = r->numParam;
  }
  else if (crtpParamTocGetItemResponse::match(result)) {
    crtpParamTocGetItemResponse* r = (crtpParamTocGetItemResponse*)result.data;
    if (r->request.id < m_paramTocEntries.size())
    {
      m_paramTocEntriesRequested.erase(r->request.id);
      ParamTocEntry& entry = m_paramTocEntries[r->request.id];
      entry.id = r->request.id;
      entry.type = (ParamType)(r->length | r-> type << 2 | r->sign << 3);
      entry.readonly = r->readonly;
      entry.group = std::string(&r->text[0]);
      entry.name = std::string(&r->text[entry.group.size() + 1]);
    }
  }
  else if (crtpParamValueResponse::match(result)) {
    crtpParamValueResponse* r = (crtpParamValueResponse*)result.data;
    ParamValue v;
    std::memcpy(&v, &r->valueFloat, 4);
    // *v = r->valueFloat;
    m_paramValues[r->id] = v;//(ParamValue)(r->valueFloat);
    m_paramValuesRequested.erase(r->id);
  }
  else if (crtpPlatformRSSIAck::match(result)) {
    crtpPlatformRSSIAck* r = (crtpPlatformRSSIAck*)result.data;
    if (m_emptyAckCallback) {
      m_emptyAckCallback(r);
    }
  }
  else if (crtpTrajectoryResponse::match(result)) {
    crtpTrajectoryResponse* r = (crtpTrajectoryResponse*)result.data;
    m_lastTrajectoryResponse = r->command;
    m_lastTrajectoryResponse2 = r->cmd1;
    m_lastTrajectoryResponse3 = r->cmd2;
  }
  else {
    crtp* header = (crtp*)result.data;
    std::cout << "Don't know ack: Port: " << (int)header->port << " Channel: " << (int)header->channel << " Len: " << (int)result.size << std::endl;
    // for (size_t i = 1; i < result.size; ++i) {
    //   std::cout << "    " << (int)result.data[i] << std::endl;
    // }
  }

}

const Crazyflie::LogTocEntry* Crazyflie::getLogTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_logTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

const Crazyflie::ParamTocEntry* Crazyflie::getParamTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_paramTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

uint8_t Crazyflie::registerLogBlock(
  std::function<void(crtpLogDataResponse*, uint8_t)> cb)
{
  for (uint8_t id = 0; id < 255; ++id) {
    if (m_logBlockCb.find(id) == m_logBlockCb.end()) {
      m_logBlockCb[id] = cb;
      return id;
    }
  }
}

bool Crazyflie::unregisterLogBlock(
  uint8_t id)
{
  m_logBlockCb.erase(m_logBlockCb.find(id));
}


////////////////////////////////////////////////////////////////

CrazyflieBroadcaster::CrazyflieBroadcaster(
  const std::string& link_uri)
  : m_radio(NULL)
  , m_devId(0)
  , m_channel(0)
  , m_address(0)
  , m_datarate(Crazyradio::Datarate_250KPS)
{
  int datarate;
  int channel;
  char datarateType;
  bool success = false;

  success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%lx",
     &m_devId, &channel, &datarate,
     &datarateType, &m_address) == 5;
  if (!success) {
    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c",
       &m_devId, &channel, &datarate,
       &datarateType) == 4;
    m_address = 0xE7E7E7E7E7;
  }

  if (success)
  {
    m_channel = channel;
    if (datarate == 250 && datarateType == 'K') {
      m_datarate = Crazyradio::Datarate_250KPS;
    }
    else if (datarate == 1 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_1MPS;
    }
    else if (datarate == 2 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_2MPS;
    }

    if (m_devId >= MAX_RADIOS) {
      throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
    }

    if (!g_crazyradios[m_devId]) {
      g_crazyradios[m_devId] = new Crazyradio(m_devId);
      // g_crazyradios[m_devId]->setAckEnable(false);
      g_crazyradios[m_devId]->setAckEnable(true);
      g_crazyradios[m_devId]->setArc(0);
    }

    m_radio = g_crazyradios[m_devId];
  }
  else {
    throw std::runtime_error("Uri is not valid!");
  }
}

void CrazyflieBroadcaster::sendPacket(
  const uint8_t* data,
  uint32_t length)
{
  {
    std::unique_lock<std::mutex> mlock(g_mutex[m_devId]);
    if (m_radio->getAddress() != m_address) {
      m_radio->setAddress(m_address);
    }
    if (m_radio->getChannel() != m_channel) {
      m_radio->setChannel(m_channel);
    }
    if (m_radio->getDatarate() != m_datarate) {
      m_radio->setDatarate(m_datarate);
    }
    if (m_radio->getAckEnable()) {
      m_radio->setAckEnable(false);
    }
    m_radio->sendPacketNoAck(data, length);
  }
}

void CrazyflieBroadcaster::trajectoryStart()
{
  crtpTrajectoryStartRequest request;
  sendPacket((const uint8_t*)&request, sizeof(request));
}

// void CrazyflieBroadcaster::setTrajectoryState(bool state)
// {
//   crtpTrajectoryStateRequest request(state);
//   sendPacket((const uint8_t*)&request, sizeof(request));
// }

void CrazyflieBroadcaster::takeoff()
{
  crtpTrajectoryTakeoffRequest request(1.2, 2000);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void CrazyflieBroadcaster::land()
{
  crtpTrajectoryLandRequest request(0.0, 2000);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void CrazyflieBroadcaster::sendPositionExternal(
  const std::vector<stateExternal>& data)
{
  crtpPosExt request;
  request.position[0].id = 0;
  request.position[1].id = 0;
  request.position[2].id = 0;
  for (size_t i = 0; i < data.size(); ++i) {
    request.position[i%3].id = data[i].id;
    request.position[i%3].x = single2half(data[i].x);
    request.position[i%3].y = single2half(data[i].y);
    request.position[i%3].z = single2half(data[i].z);
    request.position[i%3].yaw = single2half(data[i].yaw);
    if (i%3 == 2 || i == data.size() - 1) {
      sendPacket((const uint8_t*)&request, sizeof(request));
      request.position[0].id = 0;
      request.position[1].id = 0;
      request.position[2].id = 0;
    }
    // std::this_thread::sleep_for(std::chrono::microseconds(5000));
  }
}

void CrazyflieBroadcaster::sendPositionExternalBringup(
  const std::vector<stateExternalBringup>& data)
{
  crtpPosExtBringup request;
  request.pose[0].id = 0;
  request.pose[1].id = 0;
  for (size_t i = 0; i < data.size(); ++i) {
    request.pose[i%2].id = data[i].id;
    request.pose[i%2].x = single2half(data[i].x);
    request.pose[i%2].y = single2half(data[i].y);
    request.pose[i%2].z = single2half(data[i].z);
    request.pose[i%2].quat[0] = int16_t(data[i].q0 * 32768.0);
    request.pose[i%2].quat[1] = int16_t(data[i].q1 * 32768.0);
    request.pose[i%2].quat[2] = int16_t(data[i].q2 * 32768.0);
    request.pose[i%2].quat[3] = int16_t(data[i].q3 * 32768.0);
    if (i%2 == 1 || i == data.size() - 1) {
      sendPacket((const uint8_t*)&request, sizeof(request));
      request.pose[0].id = 0;
      request.pose[1].id = 0;
    }
    // std::this_thread::sleep_for(std::chrono::microseconds(5000));
  }
}
