import flask
import random
import time
import json

app = flask.Flask(__name__)

# load main page
@app.route('/')
def index():
    return flask.render_template("index.html")

# load update page
@app.route('/lidar')
def chart_data():
    def generate_random_data():
        while True:
            json_data = json.dumps(
                {'values': random.sample(range(5000,6000),360) })
            yield f"data:{json_data}\n\n"
            time.sleep(1)

    response = flask.Response(flask.stream_with_context(generate_random_data()), mimetype="text/event-stream")
    response.headers["Cache-Control"] = "no-cache"
    response.headers["X-Accel-Buffering"] = "no"
    return response

@app.route('/stop')
def stop_sensor():
    print('stop sensor')
    return "\n\n" #flask.render_template("index.html")

if __name__ == '__main__':

    # start flask server
    app.run(debug=True)
