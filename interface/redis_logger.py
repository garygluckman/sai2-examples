import redis
import time
import json
from threading import Thread


class RedisLogger(object):
    def __init__(self, redis_client):
        self.redis_client = redis_client
        self.thread = None
        self.running = False
        self.filename = ''
        self.logger_period = 0
        self.redis_keys = []

    def _logger_loop(self):
        with open(self.filename, 'w+') as f:
            f.write('Logger Frequency: {} sec\n'.format(self.logger_period))
            f.write('Time\t' +  '\t'.join(self.redis_keys) + '\n')

            while self.running:
                values = [str(self.redis_client.get(key)) for key in self.redis_keys]
                f.write(str(time.time()) + '\t' + '\t'.join(values) + '\n')
                time.sleep(self.logger_period)

    def start(self, filename, redis_keys, logger_period=1):
        if self.running:
            return False

        self.filename = filename
        self.redis_keys = redis_keys
        self.logger_period = logger_period

        # start thread
        self.running = True
        self.thread = Thread(target=self._logger_loop, daemon=True) # XXX: may not clean up correctly 
        self.thread.start()
        return True

    def stop(self):
        self.running = False
        self.thread.join()
        

### TEST ###
if __name__ == "__main__":
    r = redis.Redis()
    rl = RedisLogger(r)
    rl.start('test.log', ['sai2::sai2Interfaces::kv_pos', 'sai2::sai2Interfaces::kp_pos'], logger_period=1)
    time.sleep(5)
    rl.stop()