#!/usr/bin/env python
#This is modified from rospy/src/rospy/numpy_msg.py

import numpy
import roslib
roslib.load_manifest('rospy')
from std_msgs.msg import MultiArrayDimension

# TODO: we will need to generate a new type structure with
# little-endian specified and then pass that type structure into the
# *_numpy calls.

def _serialize_numpy(self, buff):
    """
    wrapper for factory-generated class that passes numpy module into serialize
    """
    # pass in numpy module reference to prevent import in auto-generated code
    if self.layout.dim == []:
        self.layout.dim = [ MultiArrayDimension('dim%d' %i, self.data.shape[i], self.data.shape[i]*self.data.dtype.itemsize) for i in range(len(self.data.shape))];
    self.data = self.data.reshape([1, -1])[0];
    return self.serialize_numpy(buff, numpy)

def _deserialize_numpy(self, str):
    """
    wrapper for factory-generated class that passes numpy module into deserialize    
    """
    # pass in numpy module reference to prevent import in auto-generated code
    self.deserialize_numpy(str, numpy)
    dims=map(lambda x:x.size, self.layout.dim)
    self.data = self.data.reshape(dims)
    return self
    
## Use this function to generate message instances using numpy array
## types for numerical arrays. 
## @msg_type Message class: call this functioning on the message type that you pass
## into a Publisher or Subscriber call. 
## @returns Message class
def numpy_nd_msg(msg_type):
    classdict = { '__slots__': msg_type.__slots__, '_slot_types': msg_type._slot_types,
                  '_md5sum': msg_type._md5sum, '_type': msg_type._type,
                  '_has_header': msg_type._has_header, '_full_text': msg_type._full_text,
                  'serialize': _serialize_numpy, 'deserialize': _deserialize_numpy,
                  'serialize_numpy': msg_type.serialize_numpy,
                  'deserialize_numpy': msg_type.deserialize_numpy
                  }

    # create the numpy message type
    msg_type_name = "Numpy_%s"%msg_type._type.replace('/', '__')
    return type(msg_type_name,(msg_type,),classdict)



if __name__ == '__main__':
  from std_msgs.msg import Float32MultiArray
  import rospy
  def cb(data):
     print "I heard\n",data.data

  rospy.init_node('mynode')
  pub = rospy.Publisher('mytopic', numpy_nd_msg(Float32MultiArray))
  rospy.Subscriber("mytopic", numpy_nd_msg(Float32MultiArray), cb)


  r=rospy.Rate(1);
  while not rospy.is_shutdown():
      a=numpy.array(numpy.random.randn(54,250,250), dtype=numpy.float32) #please ensure the dtype in identifical to the topic type
      print "sending\n", a
      pub.publish(data=a)
      rospy.loginfo("dimension of arr{0}".format(a.shape))
      r.sleep()
