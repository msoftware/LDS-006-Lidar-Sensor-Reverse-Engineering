#!/usr/bin/env python3
import sys, json, flask
from queue import Queue
from lds006 import LDSSerialManager

filter_array_size = 4
min_deviation = 0.02
max_deviation = 0.1

if __name__ == "__main__":

    # Fallback for broken library
    # usage: python main.py /dev/serial0 stop
    if len(sys.argv) == 3 and sys.argv[2] == "stop":
        lds = LDSSerialManager(sys.argv[1])
        lds.stop()
        sys.exit(0)
    
    # Default program
    # usage: python main.py /dev/serial0
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = "/dev/serial0"
    print("Connecting to: " + port)
    with LDSSerialManager(port) as lds:
        app = flask.Flask(__name__)
        cb_hash = None
        # load main page
        @app.route('/')
        def index():
            return flask.render_template("index.html")

        # load update page
        @app.route('/lidar')
        def chart_data(): 
            q_data = Queue(maxsize=3)
            def get_data(x):
                q_data.put(x)                                        
                
            def generate_data():
                while True:
                    json_data = json.dumps({'values': q_data.get() })
                    yield f"data:{json_data}\n\n"        

            global cb_hash
            cb_hash = lds.registerCB(get_data, range(lds.NUM_OF_ENTRIES))
            response = flask.Response(flask.stream_with_context(generate_data()), mimetype="text/event-stream")
            response.headers["Cache-Control"] = "no-cache"
            response.headers["X-Accel-Buffering"] = "no"
            return response

        @app.route('/start')
        def start_sensor():
            lds.start()
            return "\n\n"
        @app.route('/pause')
        def pause_sensor():
            lds.pause()
            return "\n\n"
        @app.route('/stop')
        def stop_sensor():
            lds.stop()
            global cb_hash
            lds.unregisterCB(cb_hash)
            return "\n\n"

        app.run(debug=True)

