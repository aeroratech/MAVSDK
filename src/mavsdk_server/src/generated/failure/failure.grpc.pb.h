// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: failure/failure.proto
#ifndef GRPC_failure_2ffailure_2eproto__INCLUDED
#define GRPC_failure_2ffailure_2eproto__INCLUDED

#include "failure/failure.pb.h"

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
namespace failure {

// Inject failures into system to test failsafes.
class FailureService final {
public:
    static constexpr char const* service_full_name() { return "mavsdk.rpc.failure.FailureService"; }
    class StubInterface {
    public:
        virtual ~StubInterface() {}
        // Injects a failure.
        virtual ::grpc::Status Inject(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::failure::InjectRequest& request,
            ::mavsdk::rpc::failure::InjectResponse* response) = 0;
        std::unique_ptr<
            ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::failure::InjectResponse>>
        AsyncInject(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::failure::InjectRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::failure::InjectResponse>>(
                AsyncInjectRaw(context, request, cq));
        }
        std::unique_ptr<
            ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::failure::InjectResponse>>
        PrepareAsyncInject(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::failure::InjectRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::failure::InjectResponse>>(
                PrepareAsyncInjectRaw(context, request, cq));
        }
        class async_interface {
        public:
            virtual ~async_interface() {}
            // Injects a failure.
            virtual void Inject(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::failure::InjectRequest* request,
                ::mavsdk::rpc::failure::InjectResponse* response,
                std::function<void(::grpc::Status)>) = 0;
            virtual void Inject(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::failure::InjectRequest* request,
                ::mavsdk::rpc::failure::InjectResponse* response,
                ::grpc::ClientUnaryReactor* reactor) = 0;
        };
        typedef class async_interface experimental_async_interface;
        virtual class async_interface* async() { return nullptr; }
        class async_interface* experimental_async() { return async(); }

    private:
        virtual ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::failure::InjectResponse>*
        AsyncInjectRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::failure::InjectRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
        virtual ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::failure::InjectResponse>*
        PrepareAsyncInjectRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::failure::InjectRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
    };
    class Stub final : public StubInterface {
    public:
        Stub(
            const std::shared_ptr<::grpc::ChannelInterface>& channel,
            const ::grpc::StubOptions& options = ::grpc::StubOptions());
        ::grpc::Status Inject(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::failure::InjectRequest& request,
            ::mavsdk::rpc::failure::InjectResponse* response) override;
        std::unique_ptr<::grpc::ClientAsyncResponseReader<::mavsdk::rpc::failure::InjectResponse>>
        AsyncInject(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::failure::InjectRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::failure::InjectResponse>>(
                AsyncInjectRaw(context, request, cq));
        }
        std::unique_ptr<::grpc::ClientAsyncResponseReader<::mavsdk::rpc::failure::InjectResponse>>
        PrepareAsyncInject(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::failure::InjectRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::failure::InjectResponse>>(
                PrepareAsyncInjectRaw(context, request, cq));
        }
        class async final : public StubInterface::async_interface {
        public:
            void Inject(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::failure::InjectRequest* request,
                ::mavsdk::rpc::failure::InjectResponse* response,
                std::function<void(::grpc::Status)>) override;
            void Inject(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::failure::InjectRequest* request,
                ::mavsdk::rpc::failure::InjectResponse* response,
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
        ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::failure::InjectResponse>* AsyncInjectRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::failure::InjectRequest& request,
            ::grpc::CompletionQueue* cq) override;
        ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::failure::InjectResponse>*
        PrepareAsyncInjectRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::failure::InjectRequest& request,
            ::grpc::CompletionQueue* cq) override;
        const ::grpc::internal::RpcMethod rpcmethod_Inject_;
    };
    static std::unique_ptr<Stub> NewStub(
        const std::shared_ptr<::grpc::ChannelInterface>& channel,
        const ::grpc::StubOptions& options = ::grpc::StubOptions());

    class Service : public ::grpc::Service {
    public:
        Service();
        virtual ~Service();
        // Injects a failure.
        virtual ::grpc::Status Inject(
            ::grpc::ServerContext* context,
            const ::mavsdk::rpc::failure::InjectRequest* request,
            ::mavsdk::rpc::failure::InjectResponse* response);
    };
    template<class BaseClass> class WithAsyncMethod_Inject : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithAsyncMethod_Inject() { ::grpc::Service::MarkMethodAsync(0); }
        ~WithAsyncMethod_Inject() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Inject(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::failure::InjectRequest* /*request*/,
            ::mavsdk::rpc::failure::InjectResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestInject(
            ::grpc::ServerContext* context,
            ::mavsdk::rpc::failure::InjectRequest* request,
            ::grpc::ServerAsyncResponseWriter<::mavsdk::rpc::failure::InjectResponse>* response,
            ::grpc::CompletionQueue* new_call_cq,
            ::grpc::ServerCompletionQueue* notification_cq,
            void* tag)
        {
            ::grpc::Service::RequestAsyncUnary(
                0, context, request, response, new_call_cq, notification_cq, tag);
        }
    };
    typedef WithAsyncMethod_Inject<Service> AsyncService;
    template<class BaseClass> class WithCallbackMethod_Inject : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithCallbackMethod_Inject()
        {
            ::grpc::Service::MarkMethodCallback(
                0,
                new ::grpc::internal::CallbackUnaryHandler<
                    ::mavsdk::rpc::failure::InjectRequest,
                    ::mavsdk::rpc::failure::InjectResponse>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::mavsdk::rpc::failure::InjectRequest* request,
                        ::mavsdk::rpc::failure::InjectResponse* response) {
                        return this->Inject(context, request, response);
                    }));
        }
        void SetMessageAllocatorFor_Inject(::grpc::MessageAllocator<
                                           ::mavsdk::rpc::failure::InjectRequest,
                                           ::mavsdk::rpc::failure::InjectResponse>* allocator)
        {
            ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
            static_cast<::grpc::internal::CallbackUnaryHandler<
                ::mavsdk::rpc::failure::InjectRequest,
                ::mavsdk::rpc::failure::InjectResponse>*>(handler)
                ->SetMessageAllocator(allocator);
        }
        ~WithCallbackMethod_Inject() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Inject(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::failure::InjectRequest* /*request*/,
            ::mavsdk::rpc::failure::InjectResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerUnaryReactor* Inject(
            ::grpc::CallbackServerContext* /*context*/,
            const ::mavsdk::rpc::failure::InjectRequest* /*request*/,
            ::mavsdk::rpc::failure::InjectResponse* /*response*/)
        {
            return nullptr;
        }
    };
    typedef WithCallbackMethod_Inject<Service> CallbackService;
    typedef CallbackService ExperimentalCallbackService;
    template<class BaseClass> class WithGenericMethod_Inject : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithGenericMethod_Inject() { ::grpc::Service::MarkMethodGeneric(0); }
        ~WithGenericMethod_Inject() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Inject(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::failure::InjectRequest* /*request*/,
            ::mavsdk::rpc::failure::InjectResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
    };
    template<class BaseClass> class WithRawMethod_Inject : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawMethod_Inject() { ::grpc::Service::MarkMethodRaw(0); }
        ~WithRawMethod_Inject() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Inject(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::failure::InjectRequest* /*request*/,
            ::mavsdk::rpc::failure::InjectResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestInject(
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
    template<class BaseClass> class WithRawCallbackMethod_Inject : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawCallbackMethod_Inject()
        {
            ::grpc::Service::MarkMethodRawCallback(
                0,
                new ::grpc::internal::CallbackUnaryHandler<::grpc::ByteBuffer, ::grpc::ByteBuffer>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::grpc::ByteBuffer* request,
                        ::grpc::ByteBuffer* response) {
                        return this->Inject(context, request, response);
                    }));
        }
        ~WithRawCallbackMethod_Inject() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status Inject(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::failure::InjectRequest* /*request*/,
            ::mavsdk::rpc::failure::InjectResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerUnaryReactor* Inject(
            ::grpc::CallbackServerContext* /*context*/,
            const ::grpc::ByteBuffer* /*request*/,
            ::grpc::ByteBuffer* /*response*/)
        {
            return nullptr;
        }
    };
    template<class BaseClass> class WithStreamedUnaryMethod_Inject : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithStreamedUnaryMethod_Inject()
        {
            ::grpc::Service::MarkMethodStreamed(
                0,
                new ::grpc::internal::StreamedUnaryHandler<
                    ::mavsdk::rpc::failure::InjectRequest,
                    ::mavsdk::rpc::failure::InjectResponse>(
                    [this](
                        ::grpc::ServerContext* context,
                        ::grpc::ServerUnaryStreamer<
                            ::mavsdk::rpc::failure::InjectRequest,
                            ::mavsdk::rpc::failure::InjectResponse>* streamer) {
                        return this->StreamedInject(context, streamer);
                    }));
        }
        ~WithStreamedUnaryMethod_Inject() override { BaseClassMustBeDerivedFromService(this); }
        // disable regular version of this method
        ::grpc::Status Inject(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::failure::InjectRequest* /*request*/,
            ::mavsdk::rpc::failure::InjectResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        // replace default version of method with streamed unary
        virtual ::grpc::Status StreamedInject(
            ::grpc::ServerContext* context,
            ::grpc::ServerUnaryStreamer<
                ::mavsdk::rpc::failure::InjectRequest,
                ::mavsdk::rpc::failure::InjectResponse>* server_unary_streamer) = 0;
    };
    typedef WithStreamedUnaryMethod_Inject<Service> StreamedUnaryService;
    typedef Service SplitStreamedService;
    typedef WithStreamedUnaryMethod_Inject<Service> StreamedService;
};

} // namespace failure
} // namespace rpc
} // namespace mavsdk

#endif // GRPC_failure_2ffailure_2eproto__INCLUDED
