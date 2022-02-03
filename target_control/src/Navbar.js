import React from 'react'
import Navbar from 'react-bootstrap/Navbar'
import Container from 'react-bootstrap/Container'
import Nav from 'react-bootstrap/Nav'
import Button from 'react-bootstrap/Button'
import Form from 'react-bootstrap/Form'
import { LinkContainer } from 'react-router-bootstrap'

function NavBar(props) {
  return (
    <Navbar bg="dark" variant="dark">
      <Container>
        <LinkContainer to="/">
          <Navbar.Brand>DroGone</Navbar.Brand>
        </LinkContainer>
        <Nav className="me-auto">
          Developer
        </Nav>
      </Container>

      <h6>
        Hostname:
      </h6>

      <Form.Control className="mx-1" style={{ width: '150px' }} size="sm" onChange={(event) => props.handleChange(event.target.value)} defaultValue={props.hostname} />
      <div className='me-2'>
        <Button variant={props.connected ? 'light' : 'warning'} onClick={() => props.handle()}>Connect</Button>
      </div>
    </Navbar>
  );
}

export default NavBar
