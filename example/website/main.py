#!/usr/bin/env python3
import sys, json, flask
from queue import Queue
from lds006.lds006 import LDSSerialManager

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
        app = flask.Flask(__name__, template_folder=".")
        cb_hash = None
        # load main page
        @app.route('/')
        def index():
            return flask.render_template("index.html")
        # load protobuf definitions
        @app.route('/msgLDS.proto')
        def pb():
            return flask.render_template("msgLDS.proto")
        # load protobuf.js sources
        @app.route('/protobuf-7.2.2.js')
        def src_protobuf():
            return flask.render_template("protobuf-7.2.2.js")
        # load protobuf.js.map sources
        @app.route('/protobuf.js.map')
        def src_protobuf2():
            return flask.render_template("protobuf.js.map")
        # load update page
        @app.route('/lidar')
        def chart_data(): 
            q_data = Queue(maxsize=3)
            def get_data(x):
                q_data.put(x)

            def generate_data():
                while True:
                    yield f"data:{q_data.get()}\n\n"        

            global cb_hash
            cb_hash = lds.registerCB(get_data)
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
            if cb_hash != None:
                lds.unregisterCB(cb_hash)
            return "\n\n"

        app.run(debug=True, threaded=True)

