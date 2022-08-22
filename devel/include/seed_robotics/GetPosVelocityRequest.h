// Generated by gencpp from file seed_robotics/GetPosVelocityRequest.msg
// DO NOT EDIT!


#ifndef SEED_ROBOTICS_MESSAGE_GETPOSVELOCITYREQUEST_H
#define SEED_ROBOTICS_MESSAGE_GETPOSVELOCITYREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace seed_robotics
{
template <class ContainerAllocator>
struct GetPosVelocityRequest_
{
  typedef GetPosVelocityRequest_<ContainerAllocator> Type;

  GetPosVelocityRequest_()
    : id(0)  {
    }
  GetPosVelocityRequest_(const ContainerAllocator& _alloc)
    : id(0)  {
  (void)_alloc;
    }



   typedef uint8_t _id_type;
  _id_type id;





  typedef boost::shared_ptr< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetPosVelocityRequest_

typedef ::seed_robotics::GetPosVelocityRequest_<std::allocator<void> > GetPosVelocityRequest;

typedef boost::shared_ptr< ::seed_robotics::GetPosVelocityRequest > GetPosVelocityRequestPtr;
typedef boost::shared_ptr< ::seed_robotics::GetPosVelocityRequest const> GetPosVelocityRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator1> & lhs, const ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator1> & lhs, const ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace seed_robotics

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "541b98e964705918fa8eb206b65347b3";
  }

  static const char* value(const ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x541b98e964705918ULL;
  static const uint64_t static_value2 = 0xfa8eb206b65347b3ULL;
};

template<class ContainerAllocator>
struct DataType< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "seed_robotics/GetPosVelocityRequest";
  }

  static const char* value(const ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 id\n"
;
  }

  static const char* value(const ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetPosVelocityRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::seed_robotics::GetPosVelocityRequest_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SEED_ROBOTICS_MESSAGE_GETPOSVELOCITYREQUEST_H
