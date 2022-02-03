import React, { Component } from 'react'
import NavBar from './Navbar';
import { BrowserRouter as Router, Switch, Route } from "react-router-dom";
import Dev from './Dev';
import ROSLIB from 'roslib';

class App extends Component {
  constructor(props) {
    super(props);
    this.handleConnection = this.handleConnection.bind(this);
    this.handleHostNameChange = this.handleHostNameChange.bind(this);
  }

  state = {
    connected: false,
    ros: new ROSLIB.Ros(),
    home: '',
    hostname: 'localhost',
  }

  handleHostNameChange(hostname) {
    this.setState({ hostname: hostname });
  }

  /**
   * Handles the connection to the webbridge node
   */
  handleConnection() {
    const ros = new ROSLIB.Ros({
      url: 'ws://' + this.state.hostname + ':9090'
    });

    console.log(ros);

    ros.on('connection', () => {
      console.log('Connected to ws server');
      this.setState({ ros: ros });
      this.setState({ connected: true });
    });

    ros.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', () => {
      console.log('Connection to websocket server closed.');
      this.setState({ connected: false });
    });
  }

  render() {
    const connected = this.state.connected;
    const ros = this.state.ros;
    return (
      <div id="outer">
        <React.StrictMode>
          <Router>
            <div id="inner_fixed">
              <NavBar connected={connected} handle={this.handleConnection} handleChange={this.handleHostNameChange} hostname={this.state.hostname} />
            </div>
            <div id="inner_remaining">
              <Switch>
                <Route path="/" render={() => (
                  <Dev ros={ros} connected={connected} home={this.state.home} />
                )} />
                <Route path="/dev" render={() => (
                  <Dev ros={ros} connected={connected} home={this.state.home} />
                )} />
              </Switch>
            </div>
          </Router>
        </React.StrictMode>
      </div>
    );
  }
}

export default App