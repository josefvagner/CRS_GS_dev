from fastapi import FastAPI

app = FastAPI()

json_sample = {
    "gps_sat": 200,
    "gps_lon": 100.2,
    "gps_lat": 100.3,
    "gps_alt": 100.3,
    "velocity": 100.5,
    "rel_alti": 234.4,
    "in_timestamp": 1364.545044,
    "out_timestamp": 1364.564087,
    "bmp_pres": 98121.734375,
    "ina_Curr": 175.699997,
    "ina_Voltage": 7.784000,
    "mpu_mag_x": -11.128282,
    "mpu_mag_y": 23.315508,
    "mpu_mag_z": 19.417095,
    "mpu_accel_x": -0.205971,
    "mpu_accel_y": -9.958491,
    "mpu_accel_z": 0.253872,
    "mpu_gyro_x": -2.288818,
    "mpu_gyro_y": -0.427246,
    "mpu_gyro_z": 0.732421,
    "fsw_state": 0,
}


def getJsonData() -> dict:
    with open("/home/gs/Desktop/CRS_GS_dev/RPI/data.bin", "rb") as fp:
        data = fp.read()
    return str(data)


@app.get("/")
def read_root():
    return getJsonData()
