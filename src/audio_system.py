#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

import rospy

from carry_my_luggage.srv import IsMeaning, SpeechToText, TextToSpeech
from speech_and_NLP.src.speechToText import recognize_speech
from speech_and_NLP.src.textToSpeech import textToSpeech

# from speech_and_NLP.src.tools.speech_to_text.isMeaning import is_meaning


class AudioSystem:
    def __init__(self):
        rospy.init_node("audio_system")
        self.tts_srv = rospy.Service("/audio_system/text_to_speech", TextToSpeech, self.tts_callback)
        self.sst_srv = rospy.Service("/audio_system/speech_to_text", SpeechToText, self.stt_callback)
        self.is_meaning_srv = rospy.Service("/audio_system/is_meaning", IsMeaning, self.is_meaning_callback)

    def tts_callback(self, msg):
        rospy.loginfo("audio_system: Speaking {}".format(msg.text))
        textToSpeech(text=msg.text)
        return True

    def stt_callback(self, msg):
        rospy.loginfo("audio_system: Listening...")
        return recognize_speech(
            msg.print_partial,
            msg.use_break,
            msg.return_extract_person_name,
            msg.remove_space,
            msg.voskLogLevel,
        )

    # TODO
    # is_meaning() is not working
    def is_meaning_callback(self, msg):
        # return is_meaning(msg.text, msg.word_list)
        pass


if __name__ == "__main__":
    try:
        audioSystem = AudioSystem()
        rospy.spin()

    except KeyboardInterrupt:
        pass
