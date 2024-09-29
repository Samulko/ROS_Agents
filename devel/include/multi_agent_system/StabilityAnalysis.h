// Generated by gencpp from file multi_agent_system/StabilityAnalysis.msg
// DO NOT EDIT!


#ifndef MULTI_AGENT_SYSTEM_MESSAGE_STABILITYANALYSIS_H
#define MULTI_AGENT_SYSTEM_MESSAGE_STABILITYANALYSIS_H

#include <ros/service_traits.h>


#include <multi_agent_system/StabilityAnalysisRequest.h>
#include <multi_agent_system/StabilityAnalysisResponse.h>


namespace multi_agent_system
{

struct StabilityAnalysis
{

typedef StabilityAnalysisRequest Request;
typedef StabilityAnalysisResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct StabilityAnalysis
} // namespace multi_agent_system


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::multi_agent_system::StabilityAnalysis > {
  static const char* value()
  {
    return "118e7fc2e317fb76cba62a2c92e6b05b";
  }

  static const char* value(const ::multi_agent_system::StabilityAnalysis&) { return value(); }
};

template<>
struct DataType< ::multi_agent_system::StabilityAnalysis > {
  static const char* value()
  {
    return "multi_agent_system/StabilityAnalysis";
  }

  static const char* value(const ::multi_agent_system::StabilityAnalysis&) { return value(); }
};


// service_traits::MD5Sum< ::multi_agent_system::StabilityAnalysisRequest> should match
// service_traits::MD5Sum< ::multi_agent_system::StabilityAnalysis >
template<>
struct MD5Sum< ::multi_agent_system::StabilityAnalysisRequest>
{
  static const char* value()
  {
    return MD5Sum< ::multi_agent_system::StabilityAnalysis >::value();
  }
  static const char* value(const ::multi_agent_system::StabilityAnalysisRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::multi_agent_system::StabilityAnalysisRequest> should match
// service_traits::DataType< ::multi_agent_system::StabilityAnalysis >
template<>
struct DataType< ::multi_agent_system::StabilityAnalysisRequest>
{
  static const char* value()
  {
    return DataType< ::multi_agent_system::StabilityAnalysis >::value();
  }
  static const char* value(const ::multi_agent_system::StabilityAnalysisRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::multi_agent_system::StabilityAnalysisResponse> should match
// service_traits::MD5Sum< ::multi_agent_system::StabilityAnalysis >
template<>
struct MD5Sum< ::multi_agent_system::StabilityAnalysisResponse>
{
  static const char* value()
  {
    return MD5Sum< ::multi_agent_system::StabilityAnalysis >::value();
  }
  static const char* value(const ::multi_agent_system::StabilityAnalysisResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::multi_agent_system::StabilityAnalysisResponse> should match
// service_traits::DataType< ::multi_agent_system::StabilityAnalysis >
template<>
struct DataType< ::multi_agent_system::StabilityAnalysisResponse>
{
  static const char* value()
  {
    return DataType< ::multi_agent_system::StabilityAnalysis >::value();
  }
  static const char* value(const ::multi_agent_system::StabilityAnalysisResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MULTI_AGENT_SYSTEM_MESSAGE_STABILITYANALYSIS_H
