from flask import Flask, jsonify, request, Response
from redis_logger import RedisLogger
import json 
import click
import redis
import catmullrom
import numpy as np

# bypass Flask templating engine by serving our HTML as static pages
app = Flask(__name__, static_folder='web', static_url_path='')


#### global variables, initialized in server start ####
redis_client = None
redis_logger = None

###########    UTILITY     ################
def get_redis_key(key):
    ''' 
    Retrieves a key from redis and attempts JSON parsing.
    We don't want to send a JSON object hiding in a string to the
    frontend, who would then be forced to double unwrap.
    '''
    redis_str = redis_client.get(key)
    try:
        return json.loads(redis_str)
    except:
        return redis_str

########### ROUTE HANDLING ################

@app.route('/')
def get_home():
    return app.send_static_file('index.html')


@app.route('/redis', methods=['GET','POST'])
def handle_redis_call():
    if request.method == 'GET':
        key_list = json.loads(request.args.get('key'))
        if type(key_list) == str:
            return jsonify(get_redis_key(key_list))
        else:
            return jsonify({key: get_redis_key(key) for key in key_list})
    elif request.method == 'POST':
        data = request.get_json()
        if type(data['val']) == list:
            redis_client.set(data['key'], json.dumps(data['val']))
        else:
            redis_client.set(data['key'], data['val'])

        return Response(status=200)


@app.route('/redis/keys', methods=['GET'])
def handle_get_all_redis_keys():
    all_keys = [key for key in redis_client.scan_iter('sai2::*')]
    all_keys.sort()
    return jsonify(all_keys)

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

@app.route('/trajectory/generate', methods=['POST'])
def handle_trajectory_generate():
    data = request.get_json()
    tf = data['tf']
    t_step = data['t_step']
    P = np.array(data['points'])
    (t_traj, P_traj, V_traj) = catmullrom.compute_catmullrom_spline_trajectory(tf, P, t_step)
    return jsonify({
        'time': t_traj.tolist(),
        'pos': P_traj.tolist(),
        'vel': V_traj.tolist()
    })

@app.route('/trajectory/run', methods=['POST'])
def handle_trajectory_run():
    pass


############ CLI + Server Init ##############
@click.group()
def server():
    pass

@server.command()
@click.option("-hp", "--http_port", help="HTTP Port (default: 8000)", default=8000, type=click.INT)
@click.option("-rh", "--redis_host", help="Redis hostname (default: localhost)", default="localhost", type=click.STRING)
@click.option("-rp", "--redis_port", help="Redis port (default: 6379)", default=6379, type=click.INT)
@click.option("-rd", "--redis_db", help="Redis database number (default: 0)", default=0, type=click.INT)
def start(http_port, redis_host, redis_port, redis_db):
    global redis_client, redis_logger
    redis_client = redis.Redis(host=redis_host, port=redis_port, db=redis_db, decode_responses=True)
    redis_logger = RedisLogger(redis_client)
    app.run(port=http_port, debug=True)


if __name__ == "__main__":
    server()