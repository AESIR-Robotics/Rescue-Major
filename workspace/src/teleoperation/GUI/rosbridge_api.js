//______________________ Rosbridge API ____________________
/**
 * RobotAPI class wraps ROSlib functionality for topic and service communication
 * with the ROS system. 
 * Dependencies: ROSLIB (must be loaded before this script)
 */

class RobotAPI {
  /**
   * Initialize the RobotAPI with a ROSLIB.Ros instance
   * @param {ROSLIB.Ros} rosInstance - Active ROS connection instance
   * @throws {Error} if rosInstance is not provided
   */
  constructor(rosInstance) {
    if (!rosInstance) {
      throw new Error('RobotAPI requires a valid ROSLIB.Ros instance');
    }
    
    this.ros = rosInstance;
    this.topicCache = {};      // { '/target_name': ROSLIB.Topic }
    this.serviceCache = {};    // { '/target_name': ROSLIB.Service }
  }

  /**
   * Get or create a topic publisher
   * @param {string} topicName - Name of the ROS topic
   * @param {string} messageType - ROS message type (e.g., 'std_msgs/String')
   * @returns {ROSLIB.Topic} The topic publisher
   */
  getOrCreateTopic(topicName, messageType) {
    if (this.topicCache[topicName]) {
      return this.topicCache[topicName];
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
      queue_size: 1
    });

    this.topicCache[topicName] = topic;
    console.log(`[RobotAPI] Created topic: ${topicName}`);
    return topic;
  }

  /**
   * Get or create a service client
   * @param {string} serviceName - Name of the ROS service
   * @param {string} serviceType - ROS service type
   * @returns {ROSLIB.Service} The service client
   */
  getOrCreateService(serviceName, serviceType) {
    if (this.serviceCache[serviceName]) {
      return this.serviceCache[serviceName];
    }

    const service = new ROSLIB.Service({
      ros: this.ros,
      name: serviceName,
      serviceType: serviceType
    });

    this.serviceCache[serviceName] = service;
    console.log(`[RobotAPI] Created service: ${serviceName}`);
    return service;
  }

  /**
   * Send message(s) to topic(s)
   * @param {string|string[]} topicNames - Single topic name or array of topic names
   * @param {string} messageType - ROS message type
   * @param {object} payload - Message payload
   * @returns {array} Array of results for each topic
   */
  sendTopic(topicNames, messageType, payload) {
    if (!Array.isArray(topicNames)) {
      topicNames = [topicNames];
    }

    const results = [];
    topicNames.forEach(topicName => {
      try {
        const topic = this.getOrCreateTopic(topicName, messageType);
        const message = new ROSLIB.Message(payload);
        topic.publish(message);
        
        console.log(`[RobotAPI] Configuración enviada a ${topicName} con estado:`, payload.state, payload);
        results.push({ success: true, target: topicName });
      } catch (error) {
        console.error(`[RobotAPI] Failed to publish to ${topicName}:`, error.message);
        results.push({ success: false, target: topicName, error: error.message });
      }
    });

    return results;
  }

  /**
   * Call ROS service(s)
   * @param {string|string[]} serviceNames - Single service name or array of service names
   * @param {string} serviceType - ROS service type
   * @param {object} payload - Service request payload
   * @returns {array} Array of results for each service call
   */
  sendService(serviceNames, serviceType, payload) {
    if (!Array.isArray(serviceNames)) {
      serviceNames = [serviceNames];
    }

    const results = [];
    serviceNames.forEach(serviceName => {
      try {
        const service = this.getOrCreateService(serviceName, serviceType);
        const request = new ROSLIB.ServiceRequest(payload);
        service.callService(request, (response) => {
          console.log(`[RobotAPI] Service ${serviceName} response:`, response);
        });
        console.log(`[RobotAPI] Called service ${serviceName} with:`, payload);
        results.push({ success: true, target: serviceName });
      } catch (error) {
        console.error(`[RobotAPI] Failed to call service ${serviceName}:`, error.message);
        results.push({ success: false, target: serviceName, error: error.message });
      }
    });

    return results;
  }

  /**
   * Get statistics about cached topics and services
   * @returns {object} Statistics object with topic and service counts
   */
  getStats() {
    return {
      topicsCached: Object.keys(this.topicCache).length,
      servicesCached: Object.keys(this.serviceCache).length
    };
  }

  /**
   * Clean up resources - unsubscribe from topics and clear caches
   */
  dispose() {
    Object.values(this.topicCache).forEach(topic => {
      try {
        topic.unsubscribe && topic.unsubscribe();
      } catch (e) {
        console.error('[RobotAPI] Error unsubscribing topic:', e);
      }
    });

    this.topicCache = {};
    this.serviceCache = {};
    console.log('[RobotAPI] Disposed');
  }
}
