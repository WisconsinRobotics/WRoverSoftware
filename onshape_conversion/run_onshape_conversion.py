import os
import subprocess
import sys

if __name__ == '__main__':
    onshape_api = 'https://cad.onshape.com'
    onshape_key = os.getenv("ONSHAPE_ACCESS_KEY")
    onshape_secret = os.getenv("ONSHAPE_SECRET_KEY")
    document_id = os.getenv("DOCUMENT_ID")
    # NOTE: This may need to be updated if the assembly name changes
    config = '{' + f"""
        "documentId": "{document_id}",
        "outputFormat": "urdf",
        "assemblyName": "ARM-PROTOTYPE",
        "simplifySTLs": "all"
    """ + '}'
    
    subprocess.run(['rm', '-f', 'converted_robot/*'])
    with open('converted_robot/config.json', 'w') as f:
        f.write(config)

    #run_converter_str = f'export ONSHAPE_ACCESS_KEY={onshape_key} && export ONSHAPE_SECRET_KEY={onshape_secret} && export ONSHAPE_API={onshape_api} && onshape-to-robot converted_robot'
    #print(run_converter_str.split())
    #subprocess.run(run_converter_str.split())

    env = {
    'ONSHAPE_ACCESS_KEY': onshape_key,
    'ONSHAPE_SECRET_KEY': onshape_secret,
    'ONSHAPE_API': onshape_api,
    **os.environ  # Include existing environment variables
    }
    subprocess.run(['onshape-to-robot', 'converted_robot'], env=env)

    subprocess.run(['rm', 'converted_robot/config.json'])
