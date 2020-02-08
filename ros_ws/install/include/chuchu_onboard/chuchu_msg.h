// Generated by gencpp from file chuchu_onboard/chuchu_msg.msg
// DO NOT EDIT!


#ifndef CHUCHU_ONBOARD_MESSAGE_CHUCHU_MSG_H
#define CHUCHU_ONBOARD_MESSAGE_CHUCHU_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace chuchu_onboard
{
template <class ContainerAllocator>
struct chuchu_msg_
{
  typedef chuchu_msg_<ContainerAllocator> Type;

  chuchu_msg_()
    : timestamp()
    , data()  {
    }
  chuchu_msg_(const ContainerAllocator& _alloc)
    : timestamp()
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Time _timestamp_type;
  _timestamp_type timestamp;

   typedef std::vector<int16_t, typename ContainerAllocator::template rebind<int16_t>::other >  _data_type;
  _data_type data;




  typedef boost::shared_ptr< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> const> ConstPtr;

}; // struct chuchu_msg_

typedef ::chuchu_onboard::chuchu_msg_<std::allocator<void> > chuchu_msg;

typedef boost::shared_ptr< ::chuchu_onboard::chuchu_msg > chuchu_msgPtr;
typedef boost::shared_ptr< ::chuchu_onboard::chuchu_msg const> chuchu_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::chuchu_onboard::chuchu_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace chuchu_onboard

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'chuchu_onboard': ['/home/grant/ros_ws/src/chuchu_onboard/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c9a1856bdb2c29d32baaf26152452c0e";
  }

  static const char* value(const ::chuchu_onboard::chuchu_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc9a1856bdb2c29d3ULL;
  static const uint64_t static_value2 = 0x2baaf26152452c0eULL;
};

template<class ContainerAllocator>
struct DataType< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "chuchu_onboard/chuchu_msg";
  }

  static const char* value(const ::chuchu_onboard::chuchu_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time timestamp\n\
int16[] data\n\
";
  }

  static const char* value(const ::chuchu_onboard::chuchu_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct chuchu_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::chuchu_onboard::chuchu_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::chuchu_onboard::chuchu_msg_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.timestamp);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CHUCHU_ONBOARD_MESSAGE_CHUCHU_MSG_H