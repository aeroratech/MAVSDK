// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: gripper/gripper.proto
#ifndef GRPC_gripper_2fgripper_2eproto__INCLUDED
#define GRPC_gripper_2fgripper_2eproto__INCLUDED

#include "gripper/gripper.pb.h"

#include <functional>
#include <grpcpp/generic/async_generic_service.h>
#include <grpcpp/support/async_stream.h>
#include <grpcpp/support/async_unary_call.h>
#include <grpcpp/support/client_callback.h>
#include <grpcpp/client_context.h>
#include <grpcpp/completion_queue.h>
#include <grpcpp/support/message_allocator.h>
#include <grpcpp/support/method_handler.h>
#include <grpcpp/impl/proto_utils.h>
#include <grpcpp/impl/rpc_method.h>
#include <grpcpp/support/server_callback.h>
#include <grpcpp/impl/server_callback_handlers.h>
#include <grpcpp/server_context.h>
#include <grpcpp/impl/service_type.h>
#include <grpcpp/support/status.h>
#include <grpcpp/support/stub_options.h>
#include <grpcpp/support/sync_stream.h>

namespace mavsdk {
namespace rpc {
namespace gripper {

//
// Allows users to send gripper actions.
class GripperService final {
public:
    static constexpr char const* service_full_name() { return "mavsdk.rpc.gripper.GripperService"; }
    class StubInterface {
    public:
        virtual ~StubInterface() {}
        //
        // Gripper grab cargo.
        virtual ::grpc::Status Grab(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest& request,
            ::mavsdk::rpc::gripper::GrabResponse* response) = 0;
        std::unique_ptr<
            ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::gripper::GrabResponse>>
        AsyncGrab(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::gripper::GrabResponse>>(
                AsyncGrabRaw(context, request, cq));
        }
        std::unique_ptr<
            ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::gripper::GrabResponse>>
        PrepareAsyncGrab(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::gripper::GrabResponse>>(
                PrepareAsyncGrabRaw(context, request, cq));
        }
        //
        // Gripper release cargo.
        virtual ::grpc::Status Release(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest& request,
            ::mavsdk::rpc::gripper::ReleaseResponse* response) = 0;
        std::unique_ptr<
            ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::gripper::ReleaseResponse>>
        AsyncRelease(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<::grpc::ClientAsyncResponseReaderInterface<
                ::mavsdk::rpc::gripper::ReleaseResponse>>(AsyncReleaseRaw(context, request, cq));
        }
        std::unique_ptr<
            ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::gripper::ReleaseResponse>>
        PrepareAsyncRelease(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<::grpc::ClientAsyncResponseReaderInterface<
                ::mavsdk::rpc::gripper::ReleaseResponse>>(
                PrepareAsyncReleaseRaw(context, request, cq));
        }
        class async_interface {
        public:
            virtual ~async_interface() {}
            //
            // Gripper grab cargo.
            virtual void Grab(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::gripper::GrabRequest* request,
                ::mavsdk::rpc::gripper::GrabResponse* response,
                std::function<void(::grpc::Status)>) = 0;
            virtual void Grab(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::gripper::GrabRequest* request,
                ::mavsdk::rpc::gripper::GrabResponse* response,
                ::grpc::ClientUnaryReactor* reactor) = 0;
            //
            // Gripper release cargo.
            virtual void Release(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::gripper::ReleaseRequest* request,
                ::mavsdk::rpc::gripper::ReleaseResponse* response,
                std::function<void(::grpc::Status)>) = 0;
            virtual void Release(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::gripper::ReleaseRequest* request,
                ::mavsdk::rpc::gripper::ReleaseResponse* response,
                ::grpc::ClientUnaryReactor* reactor) = 0;
        };
        typedef class async_interface experimental_async_interface;
        virtual class async_interface* async() { return nullptr; }
        class async_interface* experimental_async() { return async(); }

    private:
        virtual ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::gripper::GrabResponse>*
        AsyncGrabRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
        virtual ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::gripper::GrabResponse>*
        PrepareAsyncGrabRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
        virtual ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::gripper::ReleaseResponse>*
        AsyncReleaseRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
        virtual ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::gripper::ReleaseResponse>*
        PrepareAsyncReleaseRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
    };
    class Stub final : public StubInterface {
    public:
        Stub(
            const std::shared_ptr<::grpc::ChannelInterface>& channel,
            const ::grpc::StubOptions& options = ::grpc::StubOptions());
        ::grpc::Status Grab(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest& request,
            ::mavsdk::rpc::gripper::GrabResponse* response) override;
        std::unique_ptr<::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::GrabResponse>>
        AsyncGrab(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::GrabResponse>>(
                AsyncGrabRaw(context, request, cq));
        }
        std::unique_ptr<::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::GrabResponse>>
        PrepareAsyncGrab(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::GrabResponse>>(
                PrepareAsyncGrabRaw(context, request, cq));
        }
        ::grpc::Status Release(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest& request,
            ::mavsdk::rpc::gripper::ReleaseResponse* response) override;
        std::unique_ptr<::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::ReleaseResponse>>
        AsyncRelease(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::ReleaseResponse>>(
                AsyncReleaseRaw(context, request, cq));
        }
        std::unique_ptr<::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::ReleaseResponse>>
        PrepareAsyncRelease(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::ReleaseResponse>>(
                PrepareAsyncReleaseRaw(context, request, cq));
        }
        class async final : public StubInterface::async_interface {
        public:
            void Grab(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::gripper::GrabRequest* request,
                ::mavsdk::rpc::gripper::GrabResponse* response,
                std::function<void(::grpc::Status)>) override;
            void Grab(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::gripper::GrabRequest* request,
                ::mavsdk::rpc::gripper::GrabResponse* response,
                ::grpc::ClientUnaryReactor* reactor) override;
            void Release(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::gripper::ReleaseRequest* request,
                ::mavsdk::rpc::gripper::ReleaseResponse* response,
                std::function<void(::grpc::Status)>) override;
            void Release(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::gripper::ReleaseRequest* request,
                ::mavsdk::rpc::gripper::ReleaseResponse* response,
                ::grpc::ClientUnaryReactor* reactor) override;

        private:
            friend class Stub;
            explicit async(Stub* stub) : stub_(stub) {}
            Stub* stub() { return stub_; }
            Stub* stub_;
        };
        class async* async() override { return &async_stub_; }

    private:
        std::shared_ptr<::grpc::ChannelInterface> channel_;
        class async async_stub_ {
            this
        };
        ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::GrabResponse>* AsyncGrabRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest& request,
            ::grpc::CompletionQueue* cq) override;
        ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::GrabResponse>*
        PrepareAsyncGrabRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest& request,
            ::grpc::CompletionQueue* cq) override;
        ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::ReleaseResponse>* AsyncReleaseRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest& request,
            ::grpc::CompletionQueue* cq) override;
        ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::gripper::ReleaseResponse>*
        PrepareAsyncReleaseRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest& request,
            ::grpc::CompletionQueue* cq) override;
        const ::grpc::internal::RpcMethod rpcmethod_Grab_;
        const ::grpc::internal::RpcMethod rpcmethod_Release_;
    };
    static std::unique_ptr<Stub> NewStub(
        const std::shared_ptr<::grpc::ChannelInterface>& channel,
        const ::grpc::StubOptions& options = ::grpc::StubOptions());

    class Service : public ::grpc::Service {
    public:
        Service();
        virtual ~Service();
        //
        // Gripper grab cargo.
        virtual ::grpc::Status Grab(
            ::grpc::ServerContext* context,
            const ::mavsdk::rpc::gripper::GrabRequest* request,
            ::mavsdk::rpc::gripper::GrabResponse* response);
        //
        // Gripper release cargo.
        virtual ::grpc::Status Release(
            ::grpc::ServerContext* context,
            const ::mavsdk::rpc::gripper::ReleaseRequest* request,
            ::mavsdk::rpc::gripper::ReleaseResponse* response);
    };
    template<class BaseClass> class WithAsyncMethod_Grab : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithAsyncMethod_Grab() { ::grpc::Service::MarkMethodAsync(0); }
        ~WithAsyncMethod_Grab() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Grab(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::GrabRequest* /*request*/,
            ::mavsdk::rpc::gripper::GrabResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestGrab(
            ::grpc::ServerContext* context,
            ::mavsdk::rpc::gripper::GrabRequest* request,
            ::grpc::ServerAsyncResponseWriter<::mavsdk::rpc::gripper::GrabResponse>* response,
            ::grpc::CompletionQueue* new_call_cq,
            ::grpc::ServerCompletionQueue* notification_cq,
            void* tag)
        {
            ::grpc::Service::RequestAsyncUnary(
                0, context, request, response, new_call_cq, notification_cq, tag);
        }
    };
    template<class BaseClass> class WithAsyncMethod_Release : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithAsyncMethod_Release() { ::grpc::Service::MarkMethodAsync(1); }
        ~WithAsyncMethod_Release() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Release(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::ReleaseRequest* /*request*/,
            ::mavsdk::rpc::gripper::ReleaseResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestRelease(
            ::grpc::ServerContext* context,
            ::mavsdk::rpc::gripper::ReleaseRequest* request,
            ::grpc::ServerAsyncResponseWriter<::mavsdk::rpc::gripper::ReleaseResponse>* response,
            ::grpc::CompletionQueue* new_call_cq,
            ::grpc::ServerCompletionQueue* notification_cq,
            void* tag)
        {
            ::grpc::Service::RequestAsyncUnary(
                1, context, request, response, new_call_cq, notification_cq, tag);
        }
    };
    typedef WithAsyncMethod_Grab<WithAsyncMethod_Release<Service>> AsyncService;
    template<class BaseClass> class WithCallbackMethod_Grab : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithCallbackMethod_Grab()
        {
            ::grpc::Service::MarkMethodCallback(
                0,
                new ::grpc::internal::CallbackUnaryHandler<
                    ::mavsdk::rpc::gripper::GrabRequest,
                    ::mavsdk::rpc::gripper::GrabResponse>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::mavsdk::rpc::gripper::GrabRequest* request,
                        ::mavsdk::rpc::gripper::GrabResponse* response) {
                        return this->Grab(context, request, response);
                    }));
        }
        void SetMessageAllocatorFor_Grab(::grpc::MessageAllocator<
                                         ::mavsdk::rpc::gripper::GrabRequest,
                                         ::mavsdk::rpc::gripper::GrabResponse>* allocator)
        {
            ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
            static_cast<::grpc::internal::CallbackUnaryHandler<
                ::mavsdk::rpc::gripper::GrabRequest,
                ::mavsdk::rpc::gripper::GrabResponse>*>(handler)
                ->SetMessageAllocator(allocator);
        }
        ~WithCallbackMethod_Grab() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Grab(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::GrabRequest* /*request*/,
            ::mavsdk::rpc::gripper::GrabResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerUnaryReactor* Grab(
            ::grpc::CallbackServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::GrabRequest* /*request*/,
            ::mavsdk::rpc::gripper::GrabResponse* /*response*/)
        {
            return nullptr;
        }
    };
    template<class BaseClass> class WithCallbackMethod_Release : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithCallbackMethod_Release()
        {
            ::grpc::Service::MarkMethodCallback(
                1,
                new ::grpc::internal::CallbackUnaryHandler<
                    ::mavsdk::rpc::gripper::ReleaseRequest,
                    ::mavsdk::rpc::gripper::ReleaseResponse>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::mavsdk::rpc::gripper::ReleaseRequest* request,
                        ::mavsdk::rpc::gripper::ReleaseResponse* response) {
                        return this->Release(context, request, response);
                    }));
        }
        void SetMessageAllocatorFor_Release(::grpc::MessageAllocator<
                                            ::mavsdk::rpc::gripper::ReleaseRequest,
                                            ::mavsdk::rpc::gripper::ReleaseResponse>* allocator)
        {
            ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(1);
            static_cast<::grpc::internal::CallbackUnaryHandler<
                ::mavsdk::rpc::gripper::ReleaseRequest,
                ::mavsdk::rpc::gripper::ReleaseResponse>*>(handler)
                ->SetMessageAllocator(allocator);
        }
        ~WithCallbackMethod_Release() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Release(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::ReleaseRequest* /*request*/,
            ::mavsdk::rpc::gripper::ReleaseResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerUnaryReactor* Release(
            ::grpc::CallbackServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::ReleaseRequest* /*request*/,
            ::mavsdk::rpc::gripper::ReleaseResponse* /*response*/)
        {
            return nullptr;
        }
    };
    typedef WithCallbackMethod_Grab<WithCallbackMethod_Release<Service>> CallbackService;
    typedef CallbackService ExperimentalCallbackService;
    template<class BaseClass> class WithGenericMethod_Grab : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithGenericMethod_Grab() { ::grpc::Service::MarkMethodGeneric(0); }
        ~WithGenericMethod_Grab() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Grab(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::GrabRequest* /*request*/,
            ::mavsdk::rpc::gripper::GrabResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
    };
    template<class BaseClass> class WithGenericMethod_Release : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithGenericMethod_Release() { ::grpc::Service::MarkMethodGeneric(1); }
        ~WithGenericMethod_Release() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Release(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::ReleaseRequest* /*request*/,
            ::mavsdk::rpc::gripper::ReleaseResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
    };
    template<class BaseClass> class WithRawMethod_Grab : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawMethod_Grab() { ::grpc::Service::MarkMethodRaw(0); }
        ~WithRawMethod_Grab() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Grab(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::GrabRequest* /*request*/,
            ::mavsdk::rpc::gripper::GrabResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestGrab(
            ::grpc::ServerContext* context,
            ::grpc::ByteBuffer* request,
            ::grpc::ServerAsyncResponseWriter<::grpc::ByteBuffer>* response,
            ::grpc::CompletionQueue* new_call_cq,
            ::grpc::ServerCompletionQueue* notification_cq,
            void* tag)
        {
            ::grpc::Service::RequestAsyncUnary(
                0, context, request, response, new_call_cq, notification_cq, tag);
        }
    };
    template<class BaseClass> class WithRawMethod_Release : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawMethod_Release() { ::grpc::Service::MarkMethodRaw(1); }
        ~WithRawMethod_Release() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Release(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::ReleaseRequest* /*request*/,
            ::mavsdk::rpc::gripper::ReleaseResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestRelease(
            ::grpc::ServerContext* context,
            ::grpc::ByteBuffer* request,
            ::grpc::ServerAsyncResponseWriter<::grpc::ByteBuffer>* response,
            ::grpc::CompletionQueue* new_call_cq,
            ::grpc::ServerCompletionQueue* notification_cq,
            void* tag)
        {
            ::grpc::Service::RequestAsyncUnary(
                1, context, request, response, new_call_cq, notification_cq, tag);
        }
    };
    template<class BaseClass> class WithRawCallbackMethod_Grab : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawCallbackMethod_Grab()
        {
            ::grpc::Service::MarkMethodRawCallback(
                0,
                new ::grpc::internal::CallbackUnaryHandler<::grpc::ByteBuffer, ::grpc::ByteBuffer>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::grpc::ByteBuffer* request,
                        ::grpc::ByteBuffer* response) {
                        return this->Grab(context, request, response);
                    }));
        }
        ~WithRawCallbackMethod_Grab() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Grab(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::GrabRequest* /*request*/,
            ::mavsdk::rpc::gripper::GrabResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerUnaryReactor* Grab(
            ::grpc::CallbackServerContext* /*context*/,
            const ::grpc::ByteBuffer* /*request*/,
            ::grpc::ByteBuffer* /*response*/)
        {
            return nullptr;
        }
    };
    template<class BaseClass> class WithRawCallbackMethod_Release : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawCallbackMethod_Release()
        {
            ::grpc::Service::MarkMethodRawCallback(
                1,
                new ::grpc::internal::CallbackUnaryHandler<::grpc::ByteBuffer, ::grpc::ByteBuffer>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::grpc::ByteBuffer* request,
                        ::grpc::ByteBuffer* response) {
                        return this->Release(context, request, response);
                    }));
        }
        ~WithRawCallbackMethod_Release() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Release(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::ReleaseRequest* /*request*/,
            ::mavsdk::rpc::gripper::ReleaseResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerUnaryReactor* Release(
            ::grpc::CallbackServerContext* /*context*/,
            const ::grpc::ByteBuffer* /*request*/,
            ::grpc::ByteBuffer* /*response*/)
        {
            return nullptr;
        }
    };
    template<class BaseClass> class WithStreamedUnaryMethod_Grab : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithStreamedUnaryMethod_Grab()
        {
            ::grpc::Service::MarkMethodStreamed(
                0,
                new ::grpc::internal::StreamedUnaryHandler<
                    ::mavsdk::rpc::gripper::GrabRequest,
                    ::mavsdk::rpc::gripper::GrabResponse>(
                    [this](
                        ::grpc::ServerContext* context,
                        ::grpc::ServerUnaryStreamer<
                            ::mavsdk::rpc::gripper::GrabRequest,
                            ::mavsdk::rpc::gripper::GrabResponse>* streamer) {
                        return this->StreamedGrab(context, streamer);
                    }));
        }
        ~WithStreamedUnaryMethod_Grab() override { BaseClassMustBeDerivedFromService(this); }
        // disable regular version of this method
        ::grpc::Status Grab(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::GrabRequest* /*request*/,
            ::mavsdk::rpc::gripper::GrabResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        // replace default version of method with streamed unary
        virtual ::grpc::Status StreamedGrab(
            ::grpc::ServerContext* context,
            ::grpc::ServerUnaryStreamer<
                ::mavsdk::rpc::gripper::GrabRequest,
                ::mavsdk::rpc::gripper::GrabResponse>* server_unary_streamer) = 0;
    };
    template<class BaseClass> class WithStreamedUnaryMethod_Release : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithStreamedUnaryMethod_Release()
        {
            ::grpc::Service::MarkMethodStreamed(
                1,
                new ::grpc::internal::StreamedUnaryHandler<
                    ::mavsdk::rpc::gripper::ReleaseRequest,
                    ::mavsdk::rpc::gripper::ReleaseResponse>(
                    [this](
                        ::grpc::ServerContext* context,
                        ::grpc::ServerUnaryStreamer<
                            ::mavsdk::rpc::gripper::ReleaseRequest,
                            ::mavsdk::rpc::gripper::ReleaseResponse>* streamer) {
                        return this->StreamedRelease(context, streamer);
                    }));
        }
        ~WithStreamedUnaryMethod_Release() override { BaseClassMustBeDerivedFromService(this); }
        // disable regular version of this method
        ::grpc::Status Release(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::gripper::ReleaseRequest* /*request*/,
            ::mavsdk::rpc::gripper::ReleaseResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        // replace default version of method with streamed unary
        virtual ::grpc::Status StreamedRelease(
            ::grpc::ServerContext* context,
            ::grpc::ServerUnaryStreamer<
                ::mavsdk::rpc::gripper::ReleaseRequest,
                ::mavsdk::rpc::gripper::ReleaseResponse>* server_unary_streamer) = 0;
    };
    typedef WithStreamedUnaryMethod_Grab<WithStreamedUnaryMethod_Release<Service>>
        StreamedUnaryService;
    typedef Service SplitStreamedService;
    typedef WithStreamedUnaryMethod_Grab<WithStreamedUnaryMethod_Release<Service>> StreamedService;
};

} // namespace gripper
} // namespace rpc
} // namespace mavsdk

#endif // GRPC_gripper_2fgripper_2eproto__INCLUDED
