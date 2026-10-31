// ROS2 TF Tree Backend — Node.js
// Kurulum: npm install express cors
// Çalıştırma: node server.js

const express = require("express");
const cors = require("cors");
const { execFile } = require("child_process");

const app = express();
app.use(cors());

/**
 * ros2 view_frames komutunu çalıştır, frame_yaml içeriğini döndür
 */
function runTfCommand(useSimTime) {
  return new Promise((resolve) => {
    const args = ["run", "tf2_tools", "view_frames"];
    if (useSimTime) args.push("--ros-args", "-p", "use_sim_time:=true");

    execFile("ros2", args, { timeout: 15000 }, (err, stdout, stderr) => {
      const output = (stdout || "") + (stderr || "");
      const match = output.match(/frame_yaml="([\s\S]*?)"\)/);
      if (match) {
        const yaml = match[1].replace(/\\n/g, "\n").replace(/\\"/g, '"');
        resolve({ ok: true, yaml });
      } else {
        resolve({ ok: false, error: "ros2 komutu çalıştırılamadı veya TF verisi bulunamadı." });
      }
    });
  });
}

/**
 * YAML string'ini ağaç yapısına çevir
 */
function parseYamlToTree(yamlStr) {
  const frames = {};
  let currentFrame = null;

  for (const line of yamlStr.split("\n")) {
    const trimmed = line.trim();
    // "frame_name:" — girintisiz, iki nokta ile biten
    const frameMatch = line.match(/^([a-zA-Z0-9_/]+):\s*$/);
    if (frameMatch) {
      currentFrame = frameMatch[1];
      if (!frames[currentFrame]) frames[currentFrame] = { parent: null };
    } else if (currentFrame && trimmed.startsWith("parent:")) {
      const parent = trimmed.replace("parent:", "").replace(/['"\s]/g, "");
      frames[currentFrame].parent = parent || null;
      if (parent && !frames[parent]) frames[parent] = { parent: null };
    }
  }

  // Node nesneleri
  const nodes = {};
  Object.keys(frames).forEach((name) => {
    nodes[name] = { name, children: [] };
  });

  const roots = [];
  Object.keys(frames).forEach((name) => {
    const parent = frames[name].parent;
    if (parent && nodes[parent] && parent !== name) {
      nodes[parent].children.push(nodes[name]);
    } else {
      roots.push(nodes[name]);
    }
  });

  return roots;
}

// --- Routes ---

app.get("/health", (req, res) => {
  res.json({ status: "ok" });
});

app.get("/tf-tree", async (req, res) => {
  const simTime = req.query.sim_time === "true";
  const result = await runTfCommand(simTime);

  if (!result.ok) {
    return res.json({ success: false, error: result.error, tree: [] });
  }

  const tree = parseYamlToTree(result.yaml);
  if (!tree.length) {
    return res.json({ success: false, error: "TF verisi parse edilemedi.", tree: [] });
  }

  res.json({ success: true, tree });
});

// --- Başlat ---
const PORT = 8000;
app.listen(PORT, () => {
  console.log(`✓ TF Tree backend çalışıyor → http://localhost:${PORT}`);
  console.log(`  /tf-tree          — TF ağacını çek`);
  console.log(`  /tf-tree?sim_time=true  — sim_time ile çek`);
  console.log(`  /health           — sağlık kontrolü`);
});
