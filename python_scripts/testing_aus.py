import time
import zmq


if __name__ == '__main__':
    print("test plox", flush=True)
    port = "5000"

    print("Do we get live prints before ZMQ?", flush=True)
    context = zmq.Context()
    socket = context.socket(zmq.SUB)

    print( "Collecting head pose updates...", flush=True)

    socket.connect ("tcp://localhost:%s" % port)
    topic_filter = "Regression_AU06:"
    #socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)
    socket.subscribe(topic_filter)

    while True:
        au6 = socket.recv().decode('utf8')

        topic, val = au6.split(":")
        cheek_val = float(val)
        #print(socket.recv())

        #print(topic + ':' + val)
        print(topic +': %.1f' % (cheek_val), flush=True)

        time.sleep(0.01)



