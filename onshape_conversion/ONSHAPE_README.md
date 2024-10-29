First, run these commands with the keys replaced:
```
export ONSHAPE_ACCESS_KEY={our onshape key}
export ONSHAPE_SECRET_KEY={our onshape secret}
export DOCUMENT_ID={our document id}
```
Then, run this command. 
```
docker build -f onshape-converter.Dockerfile -t onshape-converter . && docker run -e ONSHAPE_ACCESS_KEY=$ONSHAPE_ACCESS_KEY -e ONSHAPE_SECRET_KEY=$ONSHAPE_SECRET_KEY -e DOCUMENT_ID=$DOCUMENT_ID -v $(pwd):/workspace onshape-converter
```
