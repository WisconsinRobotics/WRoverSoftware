import os, sys, time, io, json
from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from PIL import Image

def mark_points_on_map(driver_path, points, center = None, windowless = False):
    ## Open chrome drimer
    service = Service(driver_path)
    options = webdriver.ChromeOptions()
    if windowless:
        options.add_argument("--headless")
    else:
        options.add_experimental_option("detach", True)
    driver = webdriver.Chrome(service=service, options=options)

    ## Set center to the first point, if not provided
    if not center:
        center = points[0]

    # Open an empty page and execute JS
    driver.get(f"file://{os.path.abspath("index.html")}")
    time.sleep(2)
    result = driver.execute_script("initMap(arguments[0], arguments[1])", center, points)
    time.sleep(2)

    ## If in windowless mode, create a screenshot and save it to the image
    if windowless:
        screenshot = driver.get_screenshot_as_png()
        driver.quit()
        image = Image.open(io.BytesIO(screenshot))
        image.save("result.png")
        print(f"Image saved as result.png")


if __name__ == "__main__":
    argv = sys.argv
    if len(argv) != 3:
        print("Usage: driver_path json_path")
    
    driver_path = argv[1]
    json_path = argv[2]
    with open(argv[2]) as f:
        js = json.load(f)
    
    points = js["points"]
    center = None
    if "center" in js:
        center = js["center"]

    mark_points_on_map(driver_path, js["points"], center=center)
