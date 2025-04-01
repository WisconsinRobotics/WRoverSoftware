const script = document.createElement("script")
script.src = `https://maps.googleapis.com/maps/api/js?key=${apiKey}`;
document.head.appendChild(script);


function initMap(center, points)
{
    center = {lat: center[0], lng: center[1]}
    const map = new google.maps.Map(document.getElementById("map"), {
        center: center,
        zoom: 15,
        mapTypeId: "satellite"
    });

    for (let i = 0; i < points.length; i++)
    {
        new google.maps.Marker({
            position: {lat: points[i][0], lng: points[i][1]},
            map: map,
            title: "path point"
        })
    }
}
