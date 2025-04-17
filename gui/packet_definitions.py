PACKET_TYPES = {
    "fc_packet": { # flight computer
        "packet_type": 0x10,
        "version": 0x00,
        "payload_size": 20,
    }
}

def get_packet_info(packet_name):
    return PACKET_TYPES.get(packet_name, None) # returns None if packet not found
