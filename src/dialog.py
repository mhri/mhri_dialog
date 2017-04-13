#!/usr/bin/env python
#-*- encoding: utf8 -*-

import os
import re
import glob
import json
import threading

import rospkg
import rospy
from rivescript import RiveScript, RiveScriptError

from mhri_msgs.msg import Reply, RaisingEvents, ScriptStatus
from mhri_msgs.srv import ReloadWithResult, ReadData, WriteData

UTF8 = True
DEBUG = False
IGNORE_PUNCTATION = r'[.,!?;]'

class Dialog:
    def __init__(self):
        self.lock = threading.Lock()
        self._script_path = rospy.get_param('~script_path',
            os.path.join(rospkg.RosPack().get_path('mhri_dialog'), 'data/default_scripts'))
        self._object_path = os.path.join(rospkg.RosPack().get_path('mhri_dialog'), 'data/objects')

        self.bot = RiveScript(debug=DEBUG, utf8=UTF8)
        self.bot.unicode_punctuation = re.compile(IGNORE_PUNCTATION)
        self.bot.load_directory(self._script_path)
        self.bot.load_file(self._object_path + '/social_memory.rive')
        self.bot.sort_replies()

        self.pub_reply = rospy.Publisher('reply', Reply, queue_size=10)
        self.pub_debug_message = rospy.Publisher('script_status', ScriptStatus, queue_size=10)
        rospy.Subscriber('raising_events', RaisingEvents, self.handle_raise_events)
        self.srv_reload = rospy.Service('reload', ReloadWithResult, self.handle_reload_script)

        rospy.loginfo('[%s] Initialzed'%rospy.get_name())


    def handle_reload_script(self, req):
        reload_current = False

        if req.script_path == '':
            reload_current = True
        elif not os.path.exists(req.path):
            rospy.logerr('Requested path is not exists. Make sure the path you requested is correct.')
            return False

        self.lock.acquire()

        del self.bot
        self.bot = RiveScript(debug=DEBUG, utf8=UTF8)
        self.bot.unicode_punctuation = re.compile(IGNORE_PUNCTATION)

        if not reload_current:
            self._script_path = req.path

        self.bot.load_directory(self._script_path)
        self.bot.load_file(self._object_path + '/social_memory.rive')
        self.bot.sort_replies()

        rospy.loginfo('Reload script [%s] is succeeded.'%self._script_path)
        self.lock.release()

        return True


    def handle_raise_events(self, msg):
        reply = ''
        last_match = ''
        if msg.recognized_word != '':
            try:
                self.lock.acquire()
                reply = self.bot.reply('localuser', unicode(msg.recognized_word, 'utf-8'), errors_as_replies=False)
                last_match = self.bot.last_match('localuser')
                self.lock.release()

                reply_msg = Reply()
                reply_msg.header.stamp = rospy.Time.now()
                reply_msg.reply = reply

                self.pub_reply.publish(reply_msg)

            except RiveScriptError, e:
                rospy.logwarn('%s'%e)
                return
        else:
            for event in msg.events:
                input_event = unicode('e:'+i, 'utf-8')

                try:
                    self.lock.acquire()
                    reply = self.bot.reply('localuser', unicode(msg.recognized_word, 'utf-8'), errors_as_replies=False)
                    last_match = self.bot.last_match('localuser')
                    self.lock.release()

                    reply_msg = Reply()
                    reply_msg.header.stamp = rospy.Time.now()
                    reply_msg.reply = reply

                    self.pub_reply.publish(reply_msg)

                except RiveScriptError, e:
                    rospy.logwarn('%s'%e)
                    continue


        msg = ScriptStatus()
        msg.last_match = last_match
        msg.current_topic = self.bot.get_uservar('localuser', 'topic')
        msg.topic_structure = json.dumps(self.bot._topics)
        msg.user_vars = json.dumps(self.bot.get_uservars('localuser'))

        self.pub_debug_message.publish(msg)


if __name__ == '__main__':
	rospy.init_node('dialog', anonymous=False)
	m = Dialog()
	rospy.spin()
