You only need to run one command:
```
docker build -f onshape-converter.Dockerfile -t onshape-converter . && docker run -e ONSHAPE_ACCESS_KEY=$ONSHAPE_ACCESS_KEY -e ONSHAPE_SECRET_KEY=$ONSHAPE_SECRET_KEY -e DOCUMENT_ID=$DOCUMENT_ID -v $(pwd):/workspace onshape-converter
```
