// Generated by gencpp from file multi_agent_system/ValidateRequest.msg
// DO NOT EDIT!


#ifndef MULTI_AGENT_SYSTEM_MESSAGE_VALIDATEREQUEST_H
#define MULTI_AGENT_SYSTEM_MESSAGE_VALIDATEREQUEST_H

#include <ros/service_traits.h>


#include <multi_agent_system/ValidateRequestRequest.h>
#include <multi_agent_system/ValidateRequestResponse.h>


namespace multi_agent_system
{

struct ValidateRequest
{

typedef ValidateRequestRequest Request;
typedef ValidateRequestResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ValidateRequest
} // namespace multi_agent_system


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::multi_agent_system::ValidateRequest > {
  static const char* value()
  {
    return "d6cfda6094fa0f02393921047736849b";
  }

  static const char* value(const ::multi_agent_system::ValidateRequest&) { return value(); }
};

template<>
struct DataType< ::multi_agent_system::ValidateRequest > {
  static const char* value()
  {
    return "multi_agent_system/ValidateRequest";
  }

  static const char* value(const ::multi_agent_system::ValidateRequest&) { return value(); }
};


// service_traits::MD5Sum< ::multi_agent_system::ValidateRequestRequest> should match
// service_traits::MD5Sum< ::multi_agent_system::ValidateRequest >
template<>
struct MD5Sum< ::multi_agent_system::ValidateRequestRequest>
{
  static const char* value()
  {
    return MD5Sum< ::multi_agent_system::ValidateRequest >::value();
  }
  static const char* value(const ::multi_agent_system::ValidateRequestRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::multi_agent_system::ValidateRequestRequest> should match
// service_traits::DataType< ::multi_agent_system::ValidateRequest >
template<>
struct DataType< ::multi_agent_system::ValidateRequestRequest>
{
  static const char* value()
  {
    return DataType< ::multi_agent_system::ValidateRequest >::value();
  }
  static const char* value(const ::multi_agent_system::ValidateRequestRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::multi_agent_system::ValidateRequestResponse> should match
// service_traits::MD5Sum< ::multi_agent_system::ValidateRequest >
template<>
struct MD5Sum< ::multi_agent_system::ValidateRequestResponse>
{
  static const char* value()
  {
    return MD5Sum< ::multi_agent_system::ValidateRequest >::value();
  }
  static const char* value(const ::multi_agent_system::ValidateRequestResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::multi_agent_system::ValidateRequestResponse> should match
// service_traits::DataType< ::multi_agent_system::ValidateRequest >
template<>
struct DataType< ::multi_agent_system::ValidateRequestResponse>
{
  static const char* value()
  {
    return DataType< ::multi_agent_system::ValidateRequest >::value();
  }
  static const char* value(const ::multi_agent_system::ValidateRequestResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MULTI_AGENT_SYSTEM_MESSAGE_VALIDATEREQUEST_H
