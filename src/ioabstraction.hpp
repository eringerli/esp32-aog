// MIT License
//
// Copyright (c) 2019 Johannes Formann
// Copyright (c) 2019 Christian Riggenbach
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef IOABSTRACTION_HPP
#define IOABSTRACTION_HPP

class IoAbstractorBase {
  public:
    IoAbstractorBase() {}

  public:
    enum class WebUiIoType : uint8_t {
      Output = 0,
      Input,
      InputOutput,
      AnalogIn,
      PwmOut
    };

    typedef uint8_t IoNumber;

  public:
    // normal interactions during runtime
    virtual bool getDigitalInput( IoNumber io )  const {
      return false;
    }

    virtual uint16_t getAnalogInput( IoNumber io )  const {
      return 0;
    }

    virtual float getAnalogInputScaled( IoNumber io )  const {
      return 0.0f;
    }

    virtual void setDigitalOutput( IoNumber io, bool state ) {};
    virtual void setPwmOutput( IoNumber io, uint16_t dutyCycle ) {};

    virtual void getWebUiOptions( WebUiIoType type, uint16_t parent ) {}
};

class IoAbstraction {
  public:
    IoAbstraction( size_t numOfAbstractors ) {
      this->numOfAbstractors = numOfAbstractors;
      ports = new IoAbstractorBase*[numOfAbstractors];
    }

    void registerAbstractor( IoAbstractorBase* abstractor ) {
      static uint8_t indexInArray = 0;
      ports[indexInArray++] = abstractor;
    }

    static constexpr uint8_t maxIoPerAbstractor = 32;

  public:
    // normal interactions during runtime
    virtual bool getDigitalInput( IoAbstractorBase::IoNumber io ) const {
      return ports[io / maxIoPerAbstractor]->getDigitalInput( io % maxIoPerAbstractor );
    }
    virtual uint16_t getAnalogInput( IoAbstractorBase::IoNumber io ) const {
      return ports[io / maxIoPerAbstractor]->getAnalogInput( io % maxIoPerAbstractor );
    }
    virtual float getAnalogInputScaled( IoAbstractorBase::IoNumber io ) const {
      return ports[io / maxIoPerAbstractor]->getAnalogInputScaled( io % maxIoPerAbstractor );
    }

    virtual void setDigitalOutput( IoAbstractorBase::IoNumber io, bool state ) {
      return ports[io / maxIoPerAbstractor]->setDigitalOutput( io % maxIoPerAbstractor, state );
    };
    virtual void setPwmOutput( IoAbstractorBase::IoNumber io, uint16_t dutyCycle ) {
      return ports[io / maxIoPerAbstractor]->setPwmOutput( io % maxIoPerAbstractor, dutyCycle );
    };

    virtual void getWebUiOptions( IoAbstractorBase::WebUiIoType type, uint16_t parent ) {
      for ( uint8_t port = 0; port < numOfAbstractors; ++port ) {
        ports[port]->getWebUiOptions( type, parent );
      }
    }

  private:
    size_t numOfAbstractors = 0;
    IoAbstractorBase** ports;
};

#endif // IOABSTRACTION_HPP
