//______________________ Rosbridge API ____________________
/**
 * RobotAPI class wraps ROSlib functionality for topic and service communication
 * Dependencies: ROSLIB (must be loaded before this script)
 */

class RobotAPI {
  constructor(rosInstance) {
    if (!rosInstance) {
      throw new Error('RobotAPI requires a valid ROSLIB.Ros instance');
    }
    this.ros = rosInstance;
    this.publishers = {};
    this.services = {};
    this.subscribers = {};
  }

  // -------------------- Publishing --------------------
  getOrCreatePublisher(topicName, messageType) {
    if (this.publishers[topicName]) return this.publishers[topicName];
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
      queue_size: 1
    });
    this.publishers[topicName] = topic;
    console.log(`[RobotAPI] Created publisher for topic: ${topicName}`);
    return topic;
  }

  publishTopic(topicNames, messageType, payload) {
    if (!Array.isArray(topicNames)) topicNames = [topicNames];
    const results = [];
    topicNames.forEach(topicName => {
      try {
        const topic = this.getOrCreatePublisher(topicName, messageType);
        const message = new ROSLIB.Message(payload);
        topic.publish(message);
        console.log(`[RobotAPI] Published to ${topicName}:`, payload);
        results.push({ success: true, target: topicName });
      } catch (error) {
        console.error(`[RobotAPI] Failed to publish to ${topicName}:`, error.message);
        results.push({ success: false, target: topicName, error: error.message });
      }
    });
    return results;
  }

  // -------------------- Services --------------------
  getOrCreateService(serviceName, serviceType) {
    if (this.services[serviceName]) return this.services[serviceName];
    const service = new ROSLIB.Service({
      ros: this.ros,
      name: serviceName,
      serviceType: serviceType
    });
    this.services[serviceName] = service;
    console.log(`[RobotAPI] Created service: ${serviceName}`);
    return service;
  }

  sendService(serviceNames, serviceType, payload) {
    if (!Array.isArray(serviceNames)) serviceNames = [serviceNames];
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

  // reading topics 
  subscribeTopic(topicName, messageType, callback) {
    if (this.subscribers[topicName]) {
      console.warn(`[RobotAPI] Already subscribed to ${topicName}, skipping duplicate.`);
      return this.subscribers[topicName];
    }
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
      queue_size: 1
    });
    const subscription = {
      topic: topic,
      callback: callback || null,
      latestMessage: null
    };
    const rosCallback = (message) => {
      subscription.latestMessage = message;
      if (subscription.callback) subscription.callback(message);
      console.log(`[RobotAPI] Received message on ${topicName}:`, message);
    };
    topic.subscribe(rosCallback);
    subscription.unsubscribe = () => {
      topic.unsubscribe();
      delete this.subscribers[topicName];
      console.log(`[RobotAPI] Unsubscribed from ${topicName}`);
    };
    this.subscribers[topicName] = subscription;
    console.log(`[RobotAPI] Subscribed to ${topicName} (${messageType})`);
    return subscription;
  }

  subscribeTopics(topicsConfig) {
    if (!Array.isArray(topicsConfig)) {
      throw new Error('subscribeTopics expects an array of topic configurations');
    }
    const subscriptions = {};
    for (const config of topicsConfig) {
      const { topicName, messageType, callback } = config;
      if (!topicName || !messageType) {
        console.error('[RobotAPI] Invalid topic config:', config);
        continue;
      }
      subscriptions[topicName] = this.subscribeTopic(topicName, messageType, callback);
    }
    return subscriptions;
  }

  unsubscribeTopic(topicName) {
    const sub = this.subscribers[topicName];
    if (sub && sub.unsubscribe) {
      sub.unsubscribe();
      return true;
    }
    console.warn(`[RobotAPI] Cannot unsubscribe from ${topicName}: not subscribed.`);
    return false;
  }

  getLatestMessage(topicName) {
    const sub = this.subscribers[topicName];
    return sub ? sub.latestMessage : null;
  }

  getSubscribedTopics() {
    return Object.keys(this.subscribers).map(topicName => ({
      topicName,
      messageType: this.subscribers[topicName].topic.messageType,
      hasCallback: !!this.subscribers[topicName].callback,
      hasData: !!this.subscribers[topicName].latestMessage
    }));
  }

  
  getStats() {
    return {
      publishersCached: Object.keys(this.publishers).length,
      servicesCached: Object.keys(this.services).length,
      subscribersActive: Object.keys(this.subscribers).length
    };
  }

  dispose() {
    Object.keys(this.subscribers).forEach(topicName => this.unsubscribeTopic(topicName));
    this.publishers = {};
    this.services = {};
    this.subscribers = {};
    console.log('[RobotAPI] Disposed');
  }
}
