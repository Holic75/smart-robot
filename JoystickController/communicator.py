from enum import Enum


class ReceptionState(Enum):
    AwaitingMessage = 1,
    GettingMessageLength = 2,
    ReceivingMessage = 3,
    MessageReceived = 4


class Communicator:


    def __init__(self, channel, first_byte) :
        self._FIRST_MESSAGE_BYTE = bytes([first_byte])
        self._channel = channel 
        self._expected_length = 0
        self._state = ReceptionState.AwaitingMessage
        self._reception_buffer = bytearray([])
        self.reset()     



    def sendMessage(self, message_bytes):
        message = self._FIRST_MESSAGE_BYTE + bytearray([len(message_bytes)]) + message_bytes
        self._channel.write(message)
        self._channel.flush()


    def reset(self):
        self._state = ReceptionState.AwaitingMessage
        self._expected_length = 0
        self._reception_buffer = bytearray([])


    
    def runReceptionLoop(self):
        res = True
        while res:
            if (self._state == ReceptionState.AwaitingMessage):
                res = self._seekForIncomingMessage()

            elif (self._state == ReceptionState.GettingMessageLength):
                res = self._getIncomingMessageLength()
        
            elif (self._state == ReceptionState.ReceivingMessage):
                res = self._receiveIncomingMessage()

            elif (self._state == ReceptionState.MessageReceived):
                res = False

        return



    def _seekForIncomingMessage(self):

        while (self._channel.in_waiting):
            if (self._channel.read() == self._FIRST_MESSAGE_BYTE) :
                self._state = ReceptionState.GettingMessageLength
                return True
        
        return False



    def _getIncomingMessageLength(self):

        if (self._channel.in_waiting):
            self._expected_length = int.from_bytes(self._channel.read(), byteorder = 'little',signed = False)
            self._bytes_received = 0
            self._state = ReceptionState.ReceivingMessage
            return True

        return False



    def _receiveIncomingMessage(self):

        bytes_to_receive = self._expected_length - len(self._reception_buffer)

        if (self._channel.in_waiting and bytes_to_receive > 0):
            self._reception_buffer += self._channel.read(bytes_to_receive)

        if (len(self._reception_buffer) >= self._expected_length):
            self._state = ReceptionState.MessageReceived

        return False
    

    def isMessageReceived(self):
        return (self._state == ReceptionState.MessageReceived)


    def getReceivedMessage(self):

        if (self.isMessageReceived()):
            return self._reception_buffer.copy()
        else:
            return None