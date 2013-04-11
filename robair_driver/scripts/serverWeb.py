#!/usr/bin/env python

import datetime
import threading

import roslib
import rospy

from geventwebsocket.handler import WebSocketHandler
from gevent.pywsgi import WSGIServer
from flask import Flask, request, render_template,session, redirect

roslib.load_manifest('robair_driver')

from robair_msgs.msg import Command


class TabletNode(threading.Thread):
    '''Robair Tablet node'''
    def __init__(self, topic='/cmd', node_name="tablet"):
        self.node_name = node_name
        self.pub = rospy.Publisher(topic, Command)
        rospy.init_node('tablet')


    def key_pressed(self, key):
        if (not rospy.is_shutdown()):
            directions = {"top": (1, 0), "bottom": (-1, 0),
                      "left": (None, -90), "right": (None, 90)}
            if key in directions.keys():
                print("%s" % key)
                self.pub.publish(Command(*directions[key]))
            
tablet_node = TabletNode()
app = Flask(__name__)
app.secret_key = 'blablabla'

@app.route('/')
def reserver():
    return render_template('reservation.html')
  
@app.route('/commande')
def index():
    return render_template('navigation.html')
 
@app.route('/commande/api')
def goApi():
    api();
    
@app.route('/api')
def api():
    if request.environ.get('wsgi.websocket'):
        ws = request.environ['wsgi.websocket']
        while True:
            message = ws.receive()
            tablet_node.key_pressed(message)
            ws.send(message)
    return
 
if __name__ == '__main__':
    address = "127.0.0.1", 4243
    http_server = WSGIServer(address, app, handler_class=WebSocketHandler)
    rospy.loginfo("%s running..." % tablet_node.node_name)
    print("Server running on http://%s:%d. Ctrl+C to quit" % address)
    http_server.serve_forever()
    
