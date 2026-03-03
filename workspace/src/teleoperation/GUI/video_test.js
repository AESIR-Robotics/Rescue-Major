let _lastMerged = null;
async function loadVideo(choice, targetId = 'floating-video1') {
  try {
    const res = await fetch("static/assets/placeholder.jpg");
    if (!res.ok) throw new Error('fetch failed: ' + res.status);
    const buf = await res.arrayBuffer();
    const view = new Uint8Array(buf);

    // Buscar el marcador de fin de imagen JPG (FF D9)
    let endOfImage = -1;
    for (let i = 0; i < view.length - 1; i++) {
      if (view[i] === 0xFF && view[i + 1] === 0xD9) {
        endOfImage = i + 2;
        break;
      }
    }
    if (endOfImage === -1) throw new Error("No se encontró fin de imagen JPG");

    // Extraer el binario incrustado
    const binData = view.slice(endOfImage);

    const blockSize = 4096;
    const chunks = [];
    for (let i = 0; i < binData.length; i += blockSize * 2) {
      if (choice === 'A') chunks.push(binData.slice(i, i + blockSize));
      else chunks.push(binData.slice(i + blockSize, i + blockSize * 2));
    }

    const total = chunks.reduce((acc, c) => acc + c.length, 0);
    console.log('loadVideoFromJpg:', { choice, blocks: chunks.length, totalBytes: total });

    const merged = new Uint8Array(total);
    let offset = 0;
    for (const c of chunks) {
      merged.set(c, offset);
      offset += c.length;
    }

    _lastMerged = merged;

    const header = merged.subarray(0, 4);
    const isEbml = header.length >= 4 && header[0] === 0x1A && header[1] === 0x45 && header[2] === 0xDF && header[3] === 0xA3;

    const video = document.getElementById(targetId);
    if (!video) {
      console.error('Elemento de video no encontrado:', targetId);
      return { ok: false, reason: 'missing-element', header: Array.from(header) };
    }

    const preferred = 'video/webm; codecs="vp8, vorbis"';
    const mime = (video.canPlayType && video.canPlayType(preferred)) ? preferred : 'video/webm';

    const blob = new Blob([merged], { type: mime });
    const url = URL.createObjectURL(blob);

    if (video.dataset.blobUrl) {
      try { URL.revokeObjectURL(video.dataset.blobUrl); } catch (e) {}
    }
    video.dataset.blobUrl = url;

    video.src = url;
    video.load();

    video.onloadedmetadata = () => console.log('video loadedmetadata', { targetId, duration: video.duration });
    video.onerror = (ev) => console.error('video element error', ev);

    try { await video.play(); }
    catch (e) { console.warn('video.play() rejected', e); }

    return { ok: true, isEbml, header: Array.from(header), url };
  } catch (err) {
    console.error('loadVideoFromJpg error', err);
    throw err;
  }
}

window.loadVideo = loadVideo;
window.saveLastMerged = saveLastMerged;