# Project Setup

To use this code, make sure you have the following prerequisites:

## Prerequisites
1. **Browser Driver for Selenium**
   - Download the appropriate driver for your browser (e.g., ChromeDriver for Chrome, GeckoDriver for Firefox).
   - You can find the drivers here:
     - ChromeDriver: https://googlechromelabs.github.io/chrome-for-testing/


2. **Virtual Environment**
   - It is recommended to use a virtual environment to manage dependencies.
   - To create a virtual environment, run:
     ```bash
     python -m venv env
     ```
   - Activate the virtual environment:
     - Windows:
       ```bash
       .\env\Scripts\activate
       ```
     - macOS/Linux:
       ```bash
       source env/bin/activate
       ```
   - Install the required packages:
     ```bash
     pip install -r requirements.txt
     ```
3. **Google Maps API Key**
   - Obtain API keys for Google Maps by this link https://developers.google.com/maps/documentation/javascript/get-api-key using your account
   - In web/main.js, include your API key as in this code snippet
     ```code
     apiKey = "YOUR_API_KEY"
     ```

## Running the Code
1. Make sure the virtual environment is activated.
2. Run the script using Python:
   ```bash
   python launch.py [path to driver] [path to json]
   ```



