// Main variables for logs and peer connection
var dataChannelLog = document.getElementById('data-channel'),
    iceConnectionLog = document.getElementById('ice-connection-state'),
    iceGatheringLog = document.getElementById('ice-gathering-state'),
    signalingLog = document.getElementById('signaling-state');

var pc = null; // Variable for the PeerConnection
var dc = null, dcInterval = null; // DataChannel and its interval
let videoIndex = 0;

const videoElements = [
    document.getElementById('video1'),
    document.getElementById('video2'),
    document.getElementById('video3')
];
const placeholders = document.querySelectorAll('.placeholder');

// Function to create a new peer connection
function createPeerConnection() {
    var config = {
        sdpSemantics: 'unified-plan',
        iceServers: [
            { urls: 'stun:stun.l.google.com:19302' },  // Google STUN server for NAT traversal
            { urls: 'stun:stun1.l.google.com:19302' }, // Backup STUN server
            // Add free TURN servers for better NAT traversal
            {
                urls: 'turn:openrelay.metered.ca:80',
                username: 'openrelayproject',
                credential: 'openrelayproject'
            },
            {
                urls: 'turn:openrelay.metered.ca:443',
                username: 'openrelayproject', 
                credential: 'openrelayproject'
            }
        ]
    };

    pc = new RTCPeerConnection(config);

    // Event listeners for peer connection states
    pc.addEventListener('icegatheringstatechange', function() {
        iceGatheringLog.textContent += ' -> ' + pc.iceGatheringState;
    }, false);
    iceGatheringLog.textContent = pc.iceGatheringState;

    pc.addEventListener('iceconnectionstatechange', function() {
        iceConnectionLog.textContent += ' -> ' + pc.iceConnectionState;
    }, false);
    iceConnectionLog.textContent = pc.iceConnectionState;

    pc.addEventListener('signalingstatechange', function() {
        signalingLog.textContent += ' -> ' + pc.signalingState;
    }, false);
    signalingLog.textContent = pc.signalingState;

    pc.addEventListener('track', function(evt) {
        if (evt.track.kind === 'video') {
            if(videoIndex < videoElements.length) {
                const video = videoElements[videoIndex];
                const singleStream = new MediaStream([evt.track]);
                video.srcObject = singleStream;
                placeholders[videoIndex].style.display = 'none';
                videoIndex++;
            }
        }
        else if (evt.track.kind === 'audio') {
            const audioElement = document.getElementById('audio');
            if (audioElement) {
                const audioStream = new MediaStream([evt.track]);
                audioElement.srcObject = audioStream;
                
                // Update audio status
                const audioStatus = document.getElementById('audio-status');
                if (audioStatus) {
                    audioStatus.textContent = 'Audio: Connected';
                    audioStatus.style.color = '#28a745';
                }
                
                // Handle audio track events
                evt.track.addEventListener('ended', function() {
                    if (audioStatus) {
                        audioStatus.textContent = 'Audio: Disconnected';
                        audioStatus.style.color = '#dc3545';
                    }
                });
                
                evt.track.addEventListener('mute', function() {
                    if (audioStatus) {
                        audioStatus.textContent = 'Audio: Muted';
                        audioStatus.style.color = '#ffc107';
                    }
                });
                
                evt.track.addEventListener('unmute', function() {
                    if (audioStatus) {
                        audioStatus.textContent = 'Audio: Connected';
                        audioStatus.style.color = '#28a745';
                    }
                });
            }
        }
    });

    return pc;
}

// Enhanced autoplay handling function
function tryAutoplayAudio(audioElement) {
    // Remove any existing autoplay attributes that might interfere
    audioElement.removeAttribute('autoplay');
    
    // Try to play with different strategies
    const playPromise = audioElement.play();
    
    if (playPromise !== undefined) {
        playPromise.then(() => {
            updateAudioStatus('Audio: Playing', '#28a745');
        }).catch(error => {
            handleAutoplayBlocked(audioElement);
        });
    } else {
        // Fallback for older browsers
        setTimeout(() => {
            try {
                audioElement.play();
                updateAudioStatus('Audio: Playing', '#28a745');
            } catch (e) {
                handleAutoplayBlocked(audioElement);
            }
        }, 100);
    }
}

// Handle autoplay blocked scenarios
function handleAutoplayBlocked(audioElement) {
    updateAudioStatus('Audio: Click to enable', '#ffc107');
    
    // Show user-friendly notification
    showAudioNotification();
    
    // Add click listeners to enable audio
    addAudioEnableListeners(audioElement);
}

// Update audio status display
function updateAudioStatus(text, color) {
    const audioStatus = document.getElementById('audio-status');
    if (audioStatus) {
        audioStatus.textContent = text;
        audioStatus.style.color = color;
    }
}

// Show notification for audio enable
function showAudioNotification() {
    // Create notification element if it doesn't exist
    let notification = document.getElementById('audio-notification');
    if (!notification) {
        notification = document.createElement('div');
        notification.id = 'audio-notification';
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: #ffc107;
            color: #000;
            padding: 15px;
            border-radius: 5px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            z-index: 1000;
            font-family: 'Courier New', monospace;
            max-width: 300px;
            cursor: pointer;
            transition: opacity 0.3s ease;
        `;
        document.body.appendChild(notification);
    }
    
    notification.innerHTML = `
        🔊 Click anywhere to enable audio<br>
        <small>Required by browser security policy</small>
    `;
    notification.style.display = 'block';
    notification.style.opacity = '1';
    
    // Auto-hide after 10 seconds
    setTimeout(() => {
        if (notification.style.opacity !== '0') {
            notification.style.opacity = '0';
            setTimeout(() => {
                notification.style.display = 'none';
            }, 300);
        }
    }, 10000);
}

// Add listeners to enable audio on user interaction
function addAudioEnableListeners(audioElement) {
    // Events that can trigger audio enable
    const enableEvents = ['click', 'touchstart', 'keydown'];
    
    function enableAudio() {
        const playPromise = audioElement.play();
        if (playPromise !== undefined) {
            playPromise.then(() => {
                updateAudioStatus('Audio: Playing', '#28a745');
                hideAudioNotification();
                removeAudioEnableListeners();
            }).catch(error => {
                updateAudioStatus('Audio: Failed to enable', '#dc3545');
            });
        }
    }
    
    function removeAudioEnableListeners() {
        enableEvents.forEach(event => {
            document.removeEventListener(event, enableAudio, { capture: true });
        });
    }
    
    // Add global listeners
    enableEvents.forEach(event => {
        document.addEventListener(event, enableAudio, { capture: true, once: false });
    });
    
    // Store reference for cleanup
    audioElement._enableAudioHandler = enableAudio;
    audioElement._removeEnableListeners = removeAudioEnableListeners;
}

// Hide audio notification
function hideAudioNotification() {
    const notification = document.getElementById('audio-notification');
    if (notification) {
        notification.style.opacity = '0';
        setTimeout(() => {
            notification.style.display = 'none';
        }, 300);
    }
}

// Function to negotiate the peer connection
function negotiate()

{
    var videoResolutionSelect = document.getElementById('video-resolution');
    var selectedResolution = videoResolutionSelect.value;

    // Create offer and set local description
    return pc.createOffer().then(function(offer) {
        return pc.setLocalDescription(offer);
    }).then(function() {
        return new Promise(function(resolve) {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                function checkState() {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                }
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(function() {
        // Send the offer to the server
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: pc.localDescription.sdp,
                type: pc.localDescription.type,
                video_resolution: selectedResolution
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });

    }).then(function(response) {
        return response.json();
    }).then(function(answer) {
        return pc.setRemoteDescription(answer);
    }).catch(function(e) {
        alert(e);
    });
}

// Function to start the peer connection and data channel
function start() {

    videoIndex = 0;

    document.getElementById('start').style.display = 'none';
    pc = createPeerConnection();

    /// Reset video elements and show placeholders
    videoElements.forEach((v, i) => {
        v.srcObject = null;
        placeholders[i].style.display = 'block';
    });

    // Enable audio autoplay by user interaction
    const audioElement = document.getElementById('audio');
    if (audioElement) {
        // This user interaction enables autoplay for future audio streams
        audioElement.muted = false;
        audioElement.play().catch(e => {
        });
    }

    // Add transceivers based on server's camera count
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.addTransceiver('video', { direction: 'recvonly' });
    
    // Add audio transceiver to receive audio from server
    pc.addTransceiver('audio', { direction: 'recvonly' });

    var time_start = null;

    function current_stamp() {
        if (time_start === null) {
            time_start = new Date().getTime();
            return 0;
        } else {
            return new Date().getTime() - time_start;
        }
    }

    var parameters = JSON.parse(document.getElementById('datachannel-parameters').value);

    // Create DataChannel and setup event handlers
    dc = pc.createDataChannel('chat', parameters);
    dc.onclose = function() {
    clearInterval(dcInterval);
    dataChannelLog.textContent += '- close\n';

    };
    dc.onopen = function() {
        dataChannelLog.textContent += '- open\n';
        dcInterval = setInterval(function() {
            var message = 'ping ' + current_stamp();
            dataChannelLog.textContent += '> ' + message + '\n';
            dc.send(message);
        }, 1000);
    };
    dc.onmessage = function(evt) {
        dataChannelLog.textContent += '< ' + evt.data + '\n';

        if (evt.data.substring(0, 4) === 'pong') {
            var elapsed_ms = current_stamp() - parseInt(evt.data.substring(5), 10);
            dataChannelLog.textContent += ' RTT ' + elapsed_ms + ' ms\n';
            dc.send('latency ' + elapsed_ms);
        }
    };

    return negotiate();


}
// Function to stop the peer connection and data channel
function stop() {
    document.getElementById('stop').style.display = 'none';

    placeholders.forEach(p => p.style.display = 'block');

    if (dc) {
        dc.close();
    }

    if (pc.getTransceivers) {
        pc.getTransceivers().forEach(function(transceiver) {
            if (transceiver.stop) {
                transceiver.stop();

            }
        });
    }

    pc.getSenders().forEach(function(sender) {
        sender.track.stop();

    });

    setTimeout(function() {
        pc.close();
    }, 500);
}

// Function to filter the SDP for a specific codec
function sdpFilterCodec(kind, codec, realSdp) {
    var allowed = []
    var rtxRegex = new RegExp('a=fmtp:(\\d+) apt=(\\d+)\r$');
    var codecRegex = new RegExp('a=rtpmap:([0-9]+) ' + escapeRegExp(codec))
    var videoRegex = new RegExp('(m=' + kind + ' .*?)( ([0-9]+))*\\s*$')
    
    var lines = realSdp.split('\n');

    var isKind = false;
    for (var i = 0; i < lines.length; i++) {
        if (lines[i].startsWith('m=' + kind + ' ')) {
            isKind = true;
        } else if (lines[i].startsWith('m=')) {
            isKind = false;
        }

        if (isKind) {
            var match = lines[i].match(codecRegex);
            if (match) {
                allowed.push(parseInt(match[1]));
            }

            match = lines[i].match(rtxRegex);
            if (match && allowed.includes(parseInt(match[2]))) {
                allowed.push(parseInt(match[1]));
            }
        }
    }

    var skipRegex = 'a=(fmtp|rtcp-fb|rtpmap):([0-9]+)';
    var sdp = '';

    isKind = false;
    for (var i = 0; i < lines.length; i++) {
        if (lines[i].startsWith('m=' + kind + ' ')) {
            isKind = true;
        } else if (lines[i].startsWith('m=')) {
            isKind = false;
        }

        if (isKind) {
            var skipMatch = lines[i].match(skipRegex);
            if (skipMatch && !allowed.includes(parseInt(skipMatch[2]))) {
                continue;
            } else if (lines[i].match(videoRegex)) {
                sdp += lines[i].replace(videoRegex, '$1 ' + allowed.join(' ')) + '\n';
            } else {
                sdp += lines[i] + '\n';
            }
        } else {
            sdp += lines[i] + '\n';
        }
    }

    return sdp;
}

// Function to escape special characters in a string for RegExp
function escapeRegExp(string) {
    return string.replace(/[.*+?^${}()|[\]\\]/g, '\\$&'); 
}

// Function to toggle audio mute
function toggleAudioMute() {
    const audioElement = document.getElementById('audio');
    const muteButton = document.getElementById('mute-audio');
    const audioStatus = document.getElementById('audio-status');
    
    if (audioElement) {
        audioElement.muted = !audioElement.muted;
        
        if (audioElement.muted) {
            muteButton.textContent = 'Unmute Audio';
            if (audioStatus) {
                audioStatus.textContent = 'Audio: Muted (Local)';
                audioStatus.style.color = '#ffc107'; // Yellow for muted
            }
        } else {
            muteButton.textContent = 'Mute Audio';
            if (audioStatus) {
                audioStatus.textContent = 'Audio: Connected';
                audioStatus.style.color = '#28a745'; // Green for connected
            }
        }
    }
}

// Function to send audio control commands to server
function sendAudioCommand(command) {
    if (dc && dc.readyState === 'open') {
        dc.send('audio:' + command);
    }
}
