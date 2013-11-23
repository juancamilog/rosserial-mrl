#! /usr/bin/env python2
from xbee import ZigBee
import serial
import socket

from rosserial_python import bidirectional_node,load_message
from rosserial_msgs.msg import TopicInfo
import rospy

import threading
import sys
import time
import struct
import threading
import StringIO


serial_ports=  {}
serial_nodes = {}
nodes_in_network = {}
modem_status = '\x00'
debug = False

class FakeSerial():
    def __init__(self, xbee, node_data):
        self.rxdata = ''
        self.xbee  = xbee
        self.id = node_data['source_addr_long']
        self.lock = threading.Lock()
        self.timeout = 0.1
        self.node_data = node_data
        self.debug = rospy.get_param('~debug', False)
        
    def read(self, size = 1):
        t= 0
        counts = self.timeout/0.01
        #print "s%d   %s"%(size, self.rxdata)
        while( ( len(self.rxdata) < size ) and  (not rospy.is_shutdown()) ):
            time.sleep(0.01)
            t = t +1
            if (t >  counts):
                return ''
            
        with (self.lock):
            out = self.rxdata[:size]
            self.rxdata = self.rxdata[size:]
			
        #print "fake out " , out
        return out
		
    def write(self, data):
        if (self.debug):
            rospy.loginfo("Sending %s", [d for d in data] )
            rospy.loginfo("Sending %s bytes to %s", len(data), [d for d in self.id] )

        # fragment the packet into MAX_PACKET_SIZE chunks
        packet_size = self.xbee.MAX_PACKET_SIZE
        n_packets = len(data)/packet_size+1
        for i in xrange(n_packets):
            #TODO use fragment ids (i.e. transmit the fragment id) to check for incomplete transmissions
            fragment_id = (n_packets - i)%255
            fragment = data[i*packet_size:min((i+1)*packet_size,len(data))]
            self.xbee.send('tx', frame_id=struct.pack('B',fragment_id), options='\x01', dest_addr=self.node_data['source_addr'],dest_addr_long=self.id ,data=fragment)
            # magic sleep to avoid choking. This sleep sets a maximum rate of 16 KB/s, which is
            # above the maximum rae of 14.4 KB/s obtained at 115200 kbps
            time.sleep(0.005)
            
    def putData(self, data):
        with (self.lock):
            self.rxdata = self.rxdata+data

    def flushInput(self):
        self.rxdata = ''


def processNodeData(msg):
    data = msg['parameter']
    node_data = {}
    if msg['id'] == 'at_response':
        node_data['source_addr'] = data[0:2]
        node_data['source_addr_long'] = data[2:10]
        idx = data.find('\x00',10)
        node_data['node_id'] = data[10:idx]
        node_data['parent_source_addr'] = data[idx:idx+2]
        node_data['device_type'] = data[idx+2:idx+3]
        node_data['source_event'] = data[idx+3:idx+4]
        node_data['digi_profile_id'] = data[idx+4:idx+6]
        node_data['manufacturer_id'] = data[idx+6:idx+8]
    elif msg['id'] == 'node_id_indicator':
        node_data = msg
    # return if data is invalid    
    if len(node_data.keys())<2:
        return

    xid = node_data['source_addr_long']

    # initialize a rosserial bidirectional node for the new xbee
    rospy.loginfo("Found xbee node with address %s and id %s"%([d for d in xid], node_data['node_id']))

    if xid in nodes_in_network.keys():
        # if we already know about this node, only update node_data
        nodes_in_network[xid] = node_data
        serial_nodes[xid].requestTopics()
        return
        
    nodes_in_network[xid] = node_data
    
    serial_ports[xid] = FakeSerial(xbee, node_data)
    time.sleep(.1)
    serial_nodes[xid] = bidirectional_node.BidirectionalNode(serial_ports[xid], compressed = rospy.get_param('~compressed', False))
    initSerialNode(serial_nodes[xid],node_data)

    # start the rosserial client thread
    t = threading.Thread(target=serial_nodes[xid].run)
    t.daemon = True 
    t.start()
    return node_data

def rxCallback(msg):
    try:
        global serial_ports
        if debug:
            rospy.loginfo( "Received %s"%(msg))
        if msg['id'] == 'rx':
            src = msg['source_addr_long']
            data = msg['rf_data']
            try:
                serial_ports[src].putData(data)
            except KeyError as e:
                #rospy.loginfo("Rcv ID corrupted")
                pass
        elif msg['id'] == 'at_response':
            if msg['command'] == 'ND' and msg['status'] == '\x00':
                processNodeData(msg)
        elif msg['id'] == 'node_id_indicator':
            processNodeData(msg)
        elif msg['id'] == 'status':
            global modem_status
            modem_status = msg['status']
    except Exception, err:
    	print Exception, err
    	
    return

def initSerialNode(serial_node, node_data):
    debug = rospy.get_param('~debug', False);
    # To subscribe to atopic through rosserial means to create a local publisher that
    # forwards rosserial data to the local ROS network
    subscriber_list = rospy.get_param('~subscriber_list',[])
    topic_idx = 0
    for topic in subscriber_list:
        # only create topic if we are explicitly subscribing to this node
        # i.e topic['node_name'] == serial.node.node_data['node_id']
        #     or topic['node_name'] == 'broadcast'
        if debug:
            rospy.loginfo("Expected topic node_name [%s]. Discovered node name [%s]."%(topic['node_name'],node.node_data))
        if topic['node_name'] != node_data['node_id'] or topic['node_name'] != 'broadcast':
           pass
        if debug:
            rospy.loginfo("Trying to create serial subscriber")

        topic_idx = topic_idx+1
        msg = topic['type']
        m = load_message(msg['package'],msg['name'])

        ti = TopicInfo()
        ti.topic_id=100+topic_idx
        ti.topic_name = topic['topic']
        ti.message_type = "%s/%s"%(msg['package'],msg['name'])
        ti.md5sum  = m._md5sum
        ti.buffer_size = 512

        _buffer = StringIO.StringIO()
        ti.serialize(_buffer)
        serial_node.setupPublisher(_buffer.getvalue())
       
    # To publish to a topic through rosserial means to create a local subscriber that
    # pushes ROS messages into the rosserial link
    publisher_list = rospy.get_param('~publisher_list',[])
    for topic in publisher_list:
        # only create topic if we are explicitly publishing to this node
        # i.e topic['node_name'] == serial.node.node_data['node_id']
        #     or topic['node_name'] == 'broadcast'
        if debug:
            rospy.loginfo("Expected topic node_name [%s]. Discovered node name [%s]."%(topic['node_name'],node.node_data))
        if topic['node_name'] != node_data['node_id'] or topic['node_name'] != 'broadcast':
           pass
        if debug:
            rospy.loginfo("Trying to create serial publisher")
        topic_idx = topic_idx+1
        msg = topic['type']
        m = load_message(msg['package'],msg['name'])

        ti = TopicInfo()
        ti.topic_id=200+topic_idx
        ti.topic_name = topic['topic']
        ti.message_type = "%s/%s"%(msg['package'],msg['name'])
        ti.md5sum  = m._md5sum
        ti.buffer_size = 512

        _buffer = StringIO.StringIO()
        ti.serialize(_buffer)
        serial_node.setupSubscriber(_buffer.getvalue())

    #service_client_list = rospy.get_param('~service_client_list',[])
    #service_server_list = rospy.get_param('~service_server_list',[])
    serial_node.negotiateTopics()

def initXbeeNetwork(xbee,params):
    global modem_status
    xbee.send('at', command="SC", parameter=params['channel_mask'])
    time.sleep(0.1)
    xbee.send('at', command="ID", parameter=params['network_id'])
    time.sleep(0.1)
    xbee.send('at', command="NI", parameter=params['node_name'])
    time.sleep(0.1)
    xbee.send('at', command="AC")
    time.sleep(0.1)
    xbee.send('at', command="WR")
    time.sleep(0.1)
    
    xbee.send('at', command="FR")
    while modem_status!='\x06' and modem_status!='\x02':
        time.sleep(0.1)

    rospy.loginfo("Xbee radio started...")

    xbee.send('at', command="ND")
    time.sleep(0.1)

if __name__== '__main__':
    rospy.loginfo("RosSerial Xbee Network")
    rospy.init_node('xbee_network')

    xbee_port = rospy.get_param('~xbee_port', '/dev/ttyUSB0')
    baud_rate = rospy.get_param('~baud_rate', '9600')
    timeout = rospy.get_param('~timeout', 0.01)
    debug = rospy.get_param('~debug', False)

    # Open serial port
    ser = serial.Serial(xbee_port, baud_rate, timeout=timeout)
    rospy.loginfo("Opened Serial Port")
    ser.flush()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(0.1)

    # Create API object
    xbee = ZigBee(ser, callback= rxCallback,  escaped= True)
    rospy.loginfo("Started Xbee")
    xbee.MAX_PACKET_SIZE = 84

    # Set xbee network parameters
    xb_params = {}
    xb_params['network_id'] = rospy.get_param('~xb_network_id', '\x34\x56')
    xb_params['node_name'] = rospy.get_param('~xb_node_name', socket.gethostname())
    xb_params['channel_mask'] = rospy.get_param('~xb_channel_mask', '\x00\xFE')
    initXbeeNetwork(xbee,xb_params)

    # start listening  for other nodes in the network
    while not rospy.is_shutdown():
        try:
            # send the ND command to discover nodes
            rospy.loginfo("Checking for nodes in network...")
            xbee.send('at', command="ND")
            time.sleep(20.0)
        except KeyboardInterrupt:
            break

    #rospy.spin()
    ser.close()

    rospy.loginfo("Quiting the Sensor Network")
