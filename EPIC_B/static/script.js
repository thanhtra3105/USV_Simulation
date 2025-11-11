let map;
let waypoints = [];
let markers = [];
let polyline = null;

// Giới hạn khoảng cách (đơn vị: mét)
const MAX_DISTANCE_METERS = 1000;

// Hàm tính khoảng cách giữa 2 điểm (theo Haversine)
function distanceMeters(lat1, lon1, lat2, lon2) {
  const R = 6371000; // bán kính Trái Đất (m)
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;
  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(lat1 * Math.PI / 180) *
    Math.cos(lat2 * Math.PI / 180) *
    Math.sin(dLon / 2) * Math.sin(dLon / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
}


async function initMap() {
  try {
    // Gọi API để lấy vị trí ban đầu từ server (đọc MAVLink GPS)
    const res = await fetch("/vehicle-position");
    const data = await res.json();

    let initLat = 16.0659092;  // Giá trị mặc định nếu đọc lỗi
    let initLon = 108.1609844;

    if (data.success && data.lat && data.lon) {
      initLat = data.lat;
      initLon = data.lon;
      console.log(`Init map at GPS: ${initLat}, ${initLon}`);
    } else {
      console.warn("Không lấy được vị trí GPS, dùng tọa độ mặc định.");
    }

    // Khởi tạo Google Map tại vị trí GPS
    map = new google.maps.Map(document.getElementById("map"), {
      center: { lat: initLat, lng: initLon },
      zoom: 17,
      gestureHandling: "greedy"
    });

    // Cho phép click thêm waypoint
    map.addListener("click", (event) => {
      addWaypoint(event.latLng);
    });

  } catch (err) {
    console.error("Failed while get gps:", err);
  }
}


function addWaypoint(location) {
  // Nếu đã có ít nhất 1 waypoint
  if (waypoints.length > 0) {
    const last = waypoints[waypoints.length - 1];
    const dist = distanceMeters(last.lat, last.lng, location.lat(), location.lng());
    
    if (dist > MAX_DISTANCE_METERS) {
      alert(`This point is ${dist.toFixed(1)} m (> ${MAX_DISTANCE_METERS} m) from the previous point — invalid waypoint!!!`);
      return; // k them waypoint
    }
  }

  // Nếu hợp lệ thì thêm như bình thường
  let idx = markers.length;
  let marker = new google.maps.Marker({
    position: location,
    map: map,
    label: `${idx + 1}`,
    draggable: true
  });

  markers.push(marker);
  waypoints.push({
    lat: location.lat(),
    lng: location.lng(),
    speed: 1.0,
    hold_time: 2
  });

  marker.addListener("dragend", function(event) {
    let newPos = event.latLng;
    waypoints[idx].lat = newPos.lat();
    waypoints[idx].lng = newPos.lng();
    drawPolyline();
  });

  drawPolyline();
}

function drawPolyline() {
    if (polyline) {
        polyline.setMap(null); // xóa đường cũ
    }

    let path = waypoints.map(wp => ({ lat: wp.lat, lng: wp.lng }));

    polyline = new google.maps.Polyline({
        path: path,
        geodesic: true,
        strokeColor: "#FF0000",
        strokeOpacity: 1.0,
        strokeWeight: 2,
    });

    polyline.setMap(map);
}

function clearWaypoints() {
    markers.forEach(m => m.setMap(null));
    markers = [];
    waypoints = [];
    if (polyline) {
        polyline.setMap(null);
    }
}

function exportMission() {
    console.log(JSON.stringify({ mission: waypoints }, null, 2));
}

function updateProgress(percent, success=false) {
  const bar = document.getElementById("progress-bar");
  bar.style.width = percent + "%";
  if (success) {
    bar.style.background = "green"; // thành công thì đổi sang xanh
  } else {
    bar.style.background = "red";   //
  }
}

async function uploadMission() {
  const mission = waypoints.map(wp => ({
    lat: wp.lat,
    lng: wp.lng,
    speed: 1.0,
    hold_time: 2
  }));

  updateProgress(10); // bắt đầu
  try {
    const res = await fetch("/upload-mission", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mission })
    });
    const data = await res.json();

    if (data.success) {
      updateProgress(100, true); // full xanh khi thành công
    } else {
      updateProgress(100, false); // full đỏ nếu thất bại
      alert("Failed: " + data.message);
    }
  } catch (err) {
    updateProgress(100, false);
    alert("Error: " + err);
  }
}


// =============================
// Gọi API điều khiển vehicle
// =============================
function armVehicle() {
  fetch("/arm", { method: "POST" })
    .then(res => res.json())
    .then(data => alert("🚀 ARM: " + JSON.stringify(data)))
    .catch(err => alert("❌ ARM error: " + err));
}

function disarmVehicle() {
  fetch("/disarm", { method: "POST" })
    .then(res => res.json())
    .then(data => alert("🛑 DISARM: " + JSON.stringify(data)))
    .catch(err => alert("❌ DISARM error: " + err));
}

function startMission() {
  fetch("/start-mission", { method: "POST" })
    .then(res => res.json())
    .then(data => alert("▶️ Start Mission: " + JSON.stringify(data)))
    .catch(err => alert("❌ Start error: " + err));
}

vehicleMarker = null;

let boatIcon = {
  url: "/static/boat.png",   // icon thuyền bạn để trong thư mục static
  scaledSize: new google.maps.Size(30, 30), // kích thước
  anchor: new google.maps.Point(20, 20)     // tâm icon nằm ở giữa
};

async function updateDashboard() {
  try {
    const res = await fetch("/vehicle-info");
    const data = await res.json();

    if (data.success) {
      // Battery + Speed
      document.getElementById("battery-level").textContent = data.battery + " %";
      document.getElementById("speed").textContent = data.speed.toFixed(3) + " m/s";

      // === Cập nhật kim la bàn ===
      if (data.heading !== undefined) {
        const needle = document.getElementById("compass-needle");
        // PX4 heading: 0° là Bắc, tăng theo chiều kim đồng hồ
        // CSS rotate() cũng quay theo chiều kim đồng hồ, nên chỉ cần đảo dấu nếu gốc khác
        needle.style.transform = `translate(-50%, -50%) rotate(${data.heading}deg)`;
      }
    } else {
      document.getElementById("battery-level").textContent = "-- %";
    }
  } catch (err) {
    console.error("Battery fetch error:", err);
    document.getElementById("battery-level").textContent = "-- %";
  }
}



function updateVehiclePosition() {
  fetch("/vehicle-position")
    .then(res => res.json())
    .then(data => {
      if (data.success) {
        const pos = { lat: data.lat, lng: data.lon };

        if (!vehicleMarker) {
          vehicleMarker = new google.maps.Marker({
            position: pos,
            map: map,
            icon: boatIcon,
            title: "USV Vehicle"
          });
        } else {
          vehicleMarker.setPosition(pos);
        }
      }
    })
    .catch(err => console.error("Position update failed:", err));
}

// gọi update liên tục mỗi 2 giây
setInterval(updateVehiclePosition, 500);
setInterval(updateDashboard, 2000);
// Khởi tạo map khi load
window.onload = initMap;


