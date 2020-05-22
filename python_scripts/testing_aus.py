import time
import zmq


if __name__ == '__main__':
    print("test plox", flush=True)
    port = "5000"

    print("Do we get live prints before ZMQ?", flush=True)
    context = zmq.Context()
    regsocket = context.socket(zmq.SUB)
    classsocket = context.socket(zmq.SUB)

    print( "Collecting head pose updates...", flush=True)

    regsocket.connect ("tcp://localhost:%s" % port)
    classsocket.connect("tcp://localhost:%s" % port)
    topic_filter = "Regression"
    #socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)
    regsocket.subscribe(topic_filter)
    classsocket.subscribe("Classification")

    while True:
        reg_au = regsocket.recv().decode('utf8')
        class_au = classsocket.recv().decode('utf8')

        topic, val = reg_au.split(":")
        cheek_val = float(val)
        print(topic +': %.1f' % (cheek_val), flush=True)

        topic, val = class_au.split(":")
        cheek_val = float(val)
        print(topic +': %.1f' % (cheek_val), flush=True)

        time.sleep(0.01)



