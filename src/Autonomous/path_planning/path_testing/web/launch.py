import os, sys, time, io, json
from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from PIL import Image

def save_map_image(driver_path, html_file, output_image, center, points):
    service = Service(driver_path)
    options = webdriver.ChromeOptions()
    options.add_argument("--headless")
    driver = webdriver.Chrome(service=service, options=options)

    # Open an empty page and execute JS
    driver.get(f"file://{os.path.abspath(html_file)}")
    time.sleep(2)
    result = driver.execute_script("initMap(arguments[0], arguments[1])", center, points)
    time.sleep(2)
    screenshot = driver.get_screenshot_as_png()
    driver.quit()
    image = Image.open(io.BytesIO(screenshot))
    image.save(output_image)
    print(f"Image saved as {output_image}")

if __name__ == "__main__":
    argv = sys.argv
    if len(argv) != 3:
        print("Usage: driver_path test-number")
    
    driver_path = argv[1]
    test = argv[2]
    with open(f"../test{test}/test.json") as f:
        js = json.load(f)
    
    save_map_image(driver_path, "index.html", "result.png", js["center"], js["points"])
