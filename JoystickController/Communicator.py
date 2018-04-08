from enum import Enum


class ReceptionState(Enum):
    AwaitingMessage = 1,
    GettingMessageLength = 2,
    ReceivingMessage = 3,
    MessageReceived = 4


class Communicator:


    def __init__(self, channel, first_byte) :
        self.m_FIRST_MESSAGE_BYTE = bytes([first_byte])
        self.m_channel = channel 
        self.m_expected_length = 0
        self.m_state = ReceptionState.AwaitingMessage
        self.m_reception_buffer = bytearray([])
        self.reset()     



    def sendMessage(self, message_bytes):
        message = self.m_FIRST_MESSAGE_BYTE + bytearray([len(message_bytes)]) + message_bytes
        self.m_channel.write(message)
        self.m_channel.flush()


    def reset(self):
        self.m_state = ReceptionState.AwaitingMessage
        self.m_expected_length = 0
        self.m_reception_buffer = bytearray([])


    
    def runReceptionLoop(self):
        res = True
        while res:
            if (self.m_state == ReceptionState.AwaitingMessage):
                res = self.seekForIncomingMessage()

            elif (self.m_state == ReceptionState.GettingMessageLength):
                res = self.getIncomingMessageLength()
        
            elif (self.m_state == ReceptionState.ReceivingMessage):
                res = self.receiveIncomingMessage()

            elif (self.m_state == ReceptionState.MessageReceived):
                res = False

        return



    def seekForIncomingMessage(self):

        while (self.m_channel.in_waiting):
            if (self.m_channel.read() == self.m_FIRST_MESSAGE_BYTE) :
                self.m_state = ReceptionState.GettingMessageLength
                return True
        
        return False



    def getIncomingMessageLength(self):

        if (self.m_channel.in_waiting):
            self.m_expected_length = int.from_bytes(self.m_channel.read(), byteorder = 'little',signed = False)
            self.m_bytes_received = 0
            self.m_state = ReceptionState.ReceivingMessage
            return True

        return False



    def receiveIncomingMessage(self):

        bytes_to_receive = self.m_expected_length - len(self.m_reception_buffer)

        if (self.m_channel.in_waiting and bytes_to_receive > 0):
            self.m_reception_buffer += self.m_channel.read(bytes_to_receive)

        if (len(self.m_reception_buffer) >= self.m_expected_length):
            self.m_state = ReceptionState.MessageReceived

        return False
    

    def isMessageReceived(self):
        return (self.m_state == ReceptionState.MessageReceived)


    def getReceivedMessage(self):

        if (self.isMessageReceived()):
            return self.m_reception_buffer.copy()
        else:
            return None