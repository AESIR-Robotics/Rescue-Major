// Main variables for logs and peer connection
var dataChannelLog = document.getElementById('data-channel'),
    iceConnectionLog = document.getElementById('ice-connection-state'),
    iceGatheringLog = document.getElementById('ice-gathering-state'),
    signalingLog = document.getElementById('signaling-state');

var pc = null; // Variable for the PeerConnection
var dc = null, dcInterval = null; // DataChannel and its interval
let videoIndex = 0;
let localAudioStream = null; // Local audio stream from microphone
let remoteAudio = null; // Audio element for remote audio playback
let microphoneEnabled = false; // Microphone state (starts disabled)

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
        } else if (evt.track.kind === 'audio') {
            // Handle incoming audio from server
            console.log('Received audio track from server');
            if (!remoteAudio) {
                remoteAudio = document.createElement('audio');
                remoteAudio.autoplay = true;
                
                // Configure for low latency
                remoteAudio.volume = 1.0;
                
                // Disable buffering for minimal latency (non-standard but works in Chrome/Firefox)
                if ('latencyHint' in remoteAudio) {
                    remoteAudio.latencyHint = 0;
                }
                
                document.body.appendChild(remoteAudio);
            }
            remoteAudio.srcObject = new MediaStream([evt.track]);
            console.log('Remote audio connected and playing');
        }
    });

    return pc;
}

// Function to negotiate the peer connection
function negotiate() {
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


// Function to request microphone permission and get audio stream
async function getAudioStream() {
    try {
        const stream = await navigator.mediaDevices.getUserMedia({
            audio: {
                echoCancellation: true,
                noiseSuppression: true,
                autoGainControl: true,
                sampleRate: 48000,  // Native Opus sample rate
                channelCount: 1      // Mono
            }
        });
        
        // Verify actual configuration
        const track = stream.getAudioTracks()[0];
        const settings = track.getSettings();
        console.log('✅ Microphone configured:', settings);
        
        // Warn if not mono
        if (settings.channelCount && settings.channelCount !== 1) {
            console.warn(`⚠️ Expected mono but got ${settings.channelCount} channels`);
        }
        
        // Warn if sample rate mismatch
        if (settings.sampleRate && settings.sampleRate !== 48000) {
            console.warn(`⚠️ Expected 48kHz but got ${settings.sampleRate}Hz`);
        }
        
        return stream;
    } catch (err) {
        console.error('Microphone access denied or unavailable:', err);
        alert('Microphone access is required for audio communication. Error: ' + err.message);
        return null;
    }
}

// Toggle microphone (mute/unmute local audio track)
function toggleMicrophone() {
    const button = document.getElementById('toggle-microphone');
    
    if (!localAudioStream) {
        console.warn('No audio stream available');
        return;
    }
    
    const audioTrack = localAudioStream.getAudioTracks()[0];
    if (!audioTrack) {
        console.warn('No audio track found');
        return;
    }
    
    if (microphoneEnabled) {
        // Disable microphone locally
        audioTrack.enabled = false;
        microphoneEnabled = false;
        button.textContent = 'Enable Microphone';
        button.style.backgroundColor = '#6c757d';
        console.log('Microphone disabled ');
    } else {
        // Enable microphone locally
        audioTrack.enabled = true;
        microphoneEnabled = true;
        button.textContent = 'Disable Microphone';
        button.style.backgroundColor = '#28a745';
        console.log('Microphone enabled ');
    }
}

// Function to start the peer connection and data channel
async function start() {
    videoIndex = 0;

    document.getElementById('start').style.display = 'none';
    
    // Request microphone access first
    localAudioStream = await getAudioStream();
    
    pc = createPeerConnection();

    // Reset video elements and show placeholders
    videoElements.forEach((v, i) => {
        v.srcObject = null;
        placeholders[i].style.display = 'block';
    });

    // Add transceivers for video (receive only)
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.addTransceiver('video', { direction: 'recvonly' });

    // Add bidirectional audio transceiver
    if (localAudioStream) {
        const audioTrack = localAudioStream.getAudioTracks()[0];
        if (audioTrack) {
            // Start with microphone disabled (muted by default)
            audioTrack.enabled = false;
            microphoneEnabled = false;
            
            const audioTransceiver = pc.addTransceiver(audioTrack, { 
                direction: 'sendrecv',
                streams: [localAudioStream]
            });
            
            // Get sender and set Opus parameters for low latency
            const sender = audioTransceiver.sender;
            const parameters = sender.getParameters();
            
            // Configure Opus codec for low latency if available
            if (parameters.codecs) {
                const opusCodec = parameters.codecs.find(codec => 
                    codec.mimeType === 'audio/opus'
                );
                if (opusCodec) {
                    // Set Opus parameters for ultra-low latency
                    opusCodec.sdpFmtpLine = 'minptime=10;useinbandfec=1;maxaveragebitrate=64000';
                    parameters.codecs = [opusCodec]; // Prioritize Opus
                    sender.setParameters(parameters);
                    console.log('Opus codec configured for low latency');
                }
            }
            
            console.log('Audio transceiver added (sendrecv, initially muted)');
        }
    } else {
        console.warn('No audio stream available - audio will not be enabled');
    }

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

    // Stop local audio tracks
    if (localAudioStream) {
        localAudioStream.getTracks().forEach(track => {
            track.stop();
            console.log('Local audio track stopped');
        });
        localAudioStream = null;
    }

    // Remove remote audio element
    if (remoteAudio) {
        remoteAudio.srcObject = null;
        remoteAudio.remove();
        remoteAudio = null;
        console.log('Remote audio removed');
    }

    if (pc.getTransceivers) {
        pc.getTransceivers().forEach(function(transceiver) {
            if (transceiver.stop) {
                transceiver.stop();

            }
        });
    }

    pc.getSenders().forEach(function(sender) {
        if (sender.track) {
            sender.track.stop();
        }
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