First, run these commands with the keys replaced:
```bash
export ONSHAPE_ACCESS_KEY={our onshape key}
export ONSHAPE_SECRET_KEY={our onshape secret}
```

Then, run this command. 
```bash
docker build -f onshape-converter.Dockerfile -t onshape-converter . && docker run -e ONSHAPE_ACCESS_KEY=$ONSHAPE_ACCESS_KEY -e ONSHAPE_SECRET_KEY=$ONSHAPE_SECRET_KEY -v $(pwd):/workspace onshape-converter
```


NOTE: Suppression is something that happens in the CAD in onshape to "hide" joints. This library CANNOT deal with suppresion (see existing (issue report)[https://github.com/Rhoban/onshape-to-robot/issues/88]). So suppressions must be removed from the CAD for this to work.


Note: this is setup for the following link:
https://wirobotics.onshape.com/documents/ef0f52aab7142cfef52a3b84/w/aa6d735e54988a365f576f12/e/d87fc1d8e359617d16750c0b


