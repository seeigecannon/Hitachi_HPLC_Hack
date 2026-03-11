import socket
import pickle
import globals
import time
import struct
import select
server_socket = None
client_socket = None
import hashlib
previous_hashes = {}

def paddedLength(data, length=5):
    """
    Creates a byte object of a specified length, padded with 0s if necessary.

    Args:
        data (bytes): The initial byte data.
        length (int): The desired length of the byte object.

    Returns:
        bytes: The padded byte object.
    """
    # Ensure the input data is a byte object
    #if not isinstance(data, bytes):
    #    raise ValueError("The data must be a byte object.")

    # Pad the byte object with 0s to the desired length
    string_value = str(len(data))
    padded_string = string_value.zfill(length)
    # Return the padded byte object
    return padded_string.encode('utf-8')

def get_object_hash(obj):
    """Compute the hash of an object."""
    obj_str = repr(obj).encode('utf-8')
    return hashlib.sha256(obj_str).hexdigest()

def has_named_tuple_changed(obj):
    """
    Check if the object has changed since the last run.

    Parameters:
    - obj_name: A unique name for the object to store its hash
    - obj: The object to check for changes

    Returns:
    - True if the object has changed, False otherwise
    """
    global previous_hashes
    obj_name = obj.__class__.__name__
    # Compute the current hash of the object
    current_hash = get_object_hash(obj)

    # Get the previous hash of the object
    previous_hash = previous_hashes.get(obj_name)

    # Update the hash in the dictionary
    previous_hashes[obj_name] = current_hash

    # Check if the object has changed
    if previous_hash is None:
        # First run, no previous hash to compare
        return False
    else:
        return current_hash != previous_hash

def open_socket():
    global server_socket
    if server_socket is None:
        # Create and bind the socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('localhost', 9000))
        server_socket.listen(1)
        print("Server is listening on port 9000...")
    return server_socket

def wait_for_client_connection():
    global client_socket, server_socket
    while True:
        try:
            # Accept a new client connection
            client_socket, address = server_socket.accept()
            print(f"Connection from {address} has been established!")
            break
        except socket.error as e:
            print(f"Socket error: {e}")
            time.sleep(1)


def read_socket_data():
    """
    Reads data from a socket.

    - First checks if there's data to read using `select`.
    - Reads the first 5 bytes to determine the packet length.
    - Reads the rest of the packet based on the length.

    Parameters:
    - sock: The socket to read data from.

    Returns:
    - The complete packet as bytes if available, None otherwise.
    """
    # Use select to check if the socket has data to read
    ready_to_read, _, _ = select.select([server_socket], [], [], 0.1)

    if ready_to_read:
        # Read the first 5 bytes to get the packet length
        header = server_socket.recv(5)
        if len(header) < 5:
            return None

        packet_length = int.from_bytes(header, byteorder='big')

        # Read the remaining bytes based on the packet length
        data = server_socket.recv(packet_length)
        while len(data) < packet_length:
            data += server_socket.recv(packet_length - len(data))
        print(data)
        return data

    return None


def sendPacket(object):
    tuple_name = object.__class__.__name__
    output_str = ''.join(f"{tuple_name}. {field}:{value}\n" for field, value in zip(object._fields, object))
    output_bytes = output_str.encode('utf-8')
    #print(output_bytes)
    if has_named_tuple_changed(object):
        client_socket.sendall(paddedLength(output_bytes))
        client_socket.sendall(output_bytes)

def startServer():
    global server_socket, client_socket
    server_socket = open_socket()
    #server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #server_socket.bind(('localhost', 9000))
    #print("created socket")
    #server_socket.listen(1)
    #print("listening")
    #(client_socket, address) = server_socket.accept()
    #print("accepted socket")
    try:
        while True:
            if client_socket is None:
                wait_for_client_connection()
                print(f"client socket ={client_socket}")
                print(f"server socket{server_socket}")
            try:


                while True:
                    #for field, value in zip(globals.ovenContainer._fields, globals.ovenContainer):
                    #    client_socket.sendall(f"{field}: {value}")
                    #print(previous_hashes)
                    sendPacket(globals.ovenContainer)
                    sendPacket(globals.ASContainer)
                    sendPacket(globals.pumpContainer)
                    sendPacket(globals.ovenContainer)
                    sendPacket(globals.DADContainer)
                    sendPacket(globals.DADSpectraContainer)
                    read_socket_data()
                    #client_socket.sendall(globals.ovenContainer)
                    #client_socket.sendall(pickle.dumps(globals.ovenContainer))
                    time.sleep(.024)
                    #client_socket.sendall(pickle.dumps(globals.ovenContainer))
                    #client_socket.sendall(b'\x00')
                    #if previousOvenData != pickle.dumps(globals.ovenContainer):
                    #    previousOvenData = pickle.dumps(globals.ovenContainer)
                    #    # print(pickle.dumps(globals.ovenContainer))
                    #    client_socket.sendall(pickle.dumps(globals.ovenContainer))
            except socket.error as e:
                print(f"Socket error: {e}")
                client_socket.close()
                client_socket = None
    except KeyboardInterrupt:
        print("Server is shutting down.")
    finally:
        if client_socket is not None:
            client_socket.close()
        if server_socket is not None:
            server_socket.close()

#startServer()

#        try:
#        previousOvenData = pickle.dumps(globals.ovenContainer)
#        while True:
#            #time.sleep(1.050)
#            #client_socket.sendall(b'\x00')
###            if previousOvenData != pickle.dumps(globals.ovenContainer):
  #             previousOvenData = pickle.dumps(globals.ovenContainer)
  #             #print(pickle.dumps(globals.ovenContainer))
#               client_socket.sendall(pickle.dumps(globals.ovenContainer))#
#
#    except KeyboardInterrupt:
#        print("Server is shutting down.")
#    finally:
#        # Close the connection
#        client_socket.close()
#        server_socket.close()
