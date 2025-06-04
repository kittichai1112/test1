import requests

def send_data_to_cloud(data, endpoint_url):
    try:
        response = requests.post(endpoint_url, json=data)
        if response.status_code == 200:
            print("ส่งข้อมูลขึ้น Cloud สำเร็จ")
        else:
            print(f"เกิดข้อผิดพลาด: {response.status_code} - {response.text}")
    except Exception as e:
        print(f"ส่งข้อมูลไม่สำเร็จ: {e}")


if __name__ == "__main__":

    data = {
        "motion_detected": True,
        "breathing_rate": 15,
        "distance": 1.2,
        "spectrum": [1, 2, 3, 4, 5]  
    }
    endpoint_url = "https://your-cloud-endpoint.com/api/data"  
    send_data_to_cloud(data, endpoint_url)