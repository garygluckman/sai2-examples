from flask import Flask, jsonify, request, Response
from redis_logger import RedisLogger
import json 
import click
import redis

# bypass Flask templating engine by serving our HTML as static pages
app = Flask(__name__, static_folder='web', static_url_path='')


#### global variables, initialized in server start ####
redis_client = None
redis_logger = None

########### ROUTE HANDLING ################

@app.route('/')
def get_home():
    return app.send_static_file('index.html')


@app.route('/redis', methods=['GET','POST'])
def handle_redis_call():
    if request.method == 'GET':
        key = request.args.get('key')
        return redis_client.get(key)
    elif request.method == 'POST':
        for key in request.form:
            redis_client.set(key, request.form[key])
        return Response(status=200)


@app.route('/redis/keys', methods=['GET'])
def handle_get_all_redis_keys():
    return jsonify([key for key in redis_client.scan_iter('sai2::*')])

@app.route('/logger/status', methods=['GET'])
def handle_logger_status():
    return jsonify({'running': redis_logger.running})

@app.route('/logger/start', methods=['POST'])
def handle_logger_start():
    data = request.get_json()
    filename = data['filename']
    redis_keys = data['keys']
    logger_period = float(data['logger_period'])
    if redis_logger.start(filename, redis_keys, logger_period):
        return Response(status=200)
    else:
        return Response(status=400)

@app.route('/logger/stop', methods=['POST'])
def handle_logger_stop():
    redis_logger.stop()
    return Response(status=200)
    
@app.route('/toggle', methods=['POST'])
def handle_toggle_redis_key():
    return redis_client.keys()


############ CLI + Server Init ##############
@click.group()
def server():
    pass

@server.command()
@click.option("-hp", "--http_port", help="HTTP Port (default: 8000)", default=8000, type=click.INT)
@click.option("-wp", "--ws_port", help="WebSocket port (default: 8001)", default=8001, type=click.INT)
@click.option("-rh", "--redis_host", help="Redis hostname (default: localhost)", default="localhost", type=click.STRING)
@click.option("-rp", "--redis_port", help="Redis port (default: 6379)", default=6379, type=click.INT)
@click.option("-rd", "--redis_db", help="Redis database number (default: 0)", default=0, type=click.INT)
@click.option("-r", "--refresh_rate", help="Redis refresh rate in seconds (default: 0.05)", default=0.05, type=click.FLOAT)
def start(http_port, ws_port, redis_host, redis_port, redis_db, refresh_rate):
    global redis_client, redis_logger
    redis_client = redis.Redis(host=redis_host, port=redis_port, db=redis_db, decode_responses=True)
    redis_logger = RedisLogger(redis_client)
    app.run(port=http_port, debug=True)


if __name__ == "__main__":
    server()