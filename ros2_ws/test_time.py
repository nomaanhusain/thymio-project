import time
import datetime
start_time = time.time()
while time.time() - start_time < 10:
    print(time.time())
    now = datetime.datetime.now()
    current_time = now.strftime("%H:%M:%S")
    print(f"datetime: {current_time}")
    time.sleep(1)