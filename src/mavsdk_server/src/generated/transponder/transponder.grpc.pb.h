// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: transponder/transponder.proto
#ifndef GRPC_transponder_2ftransponder_2eproto__INCLUDED
#define GRPC_transponder_2ftransponder_2eproto__INCLUDED

#include "transponder/transponder.pb.h"

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
namespace transponder {

//
// Allow users to get ADS-B information
// and set ADS-B update rates.
class TransponderService final {
public:
    static constexpr char const* service_full_name()
    {
        return "mavsdk.rpc.transponder.TransponderService";
    }
    class StubInterface {
    public:
        virtual ~StubInterface() {}
        // Subscribe to 'transponder' updates.
        std::unique_ptr<
            ::grpc::ClientReaderInterface<::mavsdk::rpc::transponder::TransponderResponse>>
        SubscribeTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request)
        {
            return std::unique_ptr<
                ::grpc::ClientReaderInterface<::mavsdk::rpc::transponder::TransponderResponse>>(
                SubscribeTransponderRaw(context, request));
        }
        std::unique_ptr<
            ::grpc::ClientAsyncReaderInterface<::mavsdk::rpc::transponder::TransponderResponse>>
        AsyncSubscribeTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request,
            ::grpc::CompletionQueue* cq,
            void* tag)
        {
            return std::unique_ptr<::grpc::ClientAsyncReaderInterface<
                ::mavsdk::rpc::transponder::TransponderResponse>>(
                AsyncSubscribeTransponderRaw(context, request, cq, tag));
        }
        std::unique_ptr<
            ::grpc::ClientAsyncReaderInterface<::mavsdk::rpc::transponder::TransponderResponse>>
        PrepareAsyncSubscribeTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<::grpc::ClientAsyncReaderInterface<
                ::mavsdk::rpc::transponder::TransponderResponse>>(
                PrepareAsyncSubscribeTransponderRaw(context, request, cq));
        }
        // Set rate to 'transponder' updates.
        virtual ::grpc::Status SetRateTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest& request,
            ::mavsdk::rpc::transponder::SetRateTransponderResponse* response) = 0;
        std::unique_ptr<::grpc::ClientAsyncResponseReaderInterface<
            ::mavsdk::rpc::transponder::SetRateTransponderResponse>>
        AsyncSetRateTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<::grpc::ClientAsyncResponseReaderInterface<
                ::mavsdk::rpc::transponder::SetRateTransponderResponse>>(
                AsyncSetRateTransponderRaw(context, request, cq));
        }
        std::unique_ptr<::grpc::ClientAsyncResponseReaderInterface<
            ::mavsdk::rpc::transponder::SetRateTransponderResponse>>
        PrepareAsyncSetRateTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<::grpc::ClientAsyncResponseReaderInterface<
                ::mavsdk::rpc::transponder::SetRateTransponderResponse>>(
                PrepareAsyncSetRateTransponderRaw(context, request, cq));
        }
        class async_interface {
        public:
            virtual ~async_interface() {}
            // Subscribe to 'transponder' updates.
            virtual void SubscribeTransponder(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* request,
                ::grpc::ClientReadReactor<::mavsdk::rpc::transponder::TransponderResponse>*
                    reactor) = 0;
            // Set rate to 'transponder' updates.
            virtual void SetRateTransponder(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::transponder::SetRateTransponderRequest* request,
                ::mavsdk::rpc::transponder::SetRateTransponderResponse* response,
                std::function<void(::grpc::Status)>) = 0;
            virtual void SetRateTransponder(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::transponder::SetRateTransponderRequest* request,
                ::mavsdk::rpc::transponder::SetRateTransponderResponse* response,
                ::grpc::ClientUnaryReactor* reactor) = 0;
        };
        typedef class async_interface experimental_async_interface;
        virtual class async_interface* async() { return nullptr; }
        class async_interface* experimental_async() { return async(); }

    private:
        virtual ::grpc::ClientReaderInterface<::mavsdk::rpc::transponder::TransponderResponse>*
        SubscribeTransponderRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request) = 0;
        virtual ::grpc::ClientAsyncReaderInterface<::mavsdk::rpc::transponder::TransponderResponse>*
        AsyncSubscribeTransponderRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request,
            ::grpc::CompletionQueue* cq,
            void* tag) = 0;
        virtual ::grpc::ClientAsyncReaderInterface<::mavsdk::rpc::transponder::TransponderResponse>*
        PrepareAsyncSubscribeTransponderRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
        virtual ::grpc::ClientAsyncResponseReaderInterface<
            ::mavsdk::rpc::transponder::SetRateTransponderResponse>*
        AsyncSetRateTransponderRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
        virtual ::grpc::ClientAsyncResponseReaderInterface<
            ::mavsdk::rpc::transponder::SetRateTransponderResponse>*
        PrepareAsyncSetRateTransponderRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
    };
    class Stub final : public StubInterface {
    public:
        Stub(
            const std::shared_ptr<::grpc::ChannelInterface>& channel,
            const ::grpc::StubOptions& options = ::grpc::StubOptions());
        std::unique_ptr<::grpc::ClientReader<::mavsdk::rpc::transponder::TransponderResponse>>
        SubscribeTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request)
        {
            return std::unique_ptr<
                ::grpc::ClientReader<::mavsdk::rpc::transponder::TransponderResponse>>(
                SubscribeTransponderRaw(context, request));
        }
        std::unique_ptr<::grpc::ClientAsyncReader<::mavsdk::rpc::transponder::TransponderResponse>>
        AsyncSubscribeTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request,
            ::grpc::CompletionQueue* cq,
            void* tag)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncReader<::mavsdk::rpc::transponder::TransponderResponse>>(
                AsyncSubscribeTransponderRaw(context, request, cq, tag));
        }
        std::unique_ptr<::grpc::ClientAsyncReader<::mavsdk::rpc::transponder::TransponderResponse>>
        PrepareAsyncSubscribeTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncReader<::mavsdk::rpc::transponder::TransponderResponse>>(
                PrepareAsyncSubscribeTransponderRaw(context, request, cq));
        }
        ::grpc::Status SetRateTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest& request,
            ::mavsdk::rpc::transponder::SetRateTransponderResponse* response) override;
        std::unique_ptr<::grpc::ClientAsyncResponseReader<
            ::mavsdk::rpc::transponder::SetRateTransponderResponse>>
        AsyncSetRateTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<::grpc::ClientAsyncResponseReader<
                ::mavsdk::rpc::transponder::SetRateTransponderResponse>>(
                AsyncSetRateTransponderRaw(context, request, cq));
        }
        std::unique_ptr<::grpc::ClientAsyncResponseReader<
            ::mavsdk::rpc::transponder::SetRateTransponderResponse>>
        PrepareAsyncSetRateTransponder(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<::grpc::ClientAsyncResponseReader<
                ::mavsdk::rpc::transponder::SetRateTransponderResponse>>(
                PrepareAsyncSetRateTransponderRaw(context, request, cq));
        }
        class async final : public StubInterface::async_interface {
        public:
            void SubscribeTransponder(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* request,
                ::grpc::ClientReadReactor<::mavsdk::rpc::transponder::TransponderResponse>* reactor)
                override;
            void SetRateTransponder(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::transponder::SetRateTransponderRequest* request,
                ::mavsdk::rpc::transponder::SetRateTransponderResponse* response,
                std::function<void(::grpc::Status)>) override;
            void SetRateTransponder(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::transponder::SetRateTransponderRequest* request,
                ::mavsdk::rpc::transponder::SetRateTransponderResponse* response,
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
        ::grpc::ClientReader<::mavsdk::rpc::transponder::TransponderResponse>*
        SubscribeTransponderRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request) override;
        ::grpc::ClientAsyncReader<::mavsdk::rpc::transponder::TransponderResponse>*
        AsyncSubscribeTransponderRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request,
            ::grpc::CompletionQueue* cq,
            void* tag) override;
        ::grpc::ClientAsyncReader<::mavsdk::rpc::transponder::TransponderResponse>*
        PrepareAsyncSubscribeTransponderRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest& request,
            ::grpc::CompletionQueue* cq) override;
        ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::transponder::SetRateTransponderResponse>*
        AsyncSetRateTransponderRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest& request,
            ::grpc::CompletionQueue* cq) override;
        ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::transponder::SetRateTransponderResponse>*
        PrepareAsyncSetRateTransponderRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest& request,
            ::grpc::CompletionQueue* cq) override;
        const ::grpc::internal::RpcMethod rpcmethod_SubscribeTransponder_;
        const ::grpc::internal::RpcMethod rpcmethod_SetRateTransponder_;
    };
    static std::unique_ptr<Stub> NewStub(
        const std::shared_ptr<::grpc::ChannelInterface>& channel,
        const ::grpc::StubOptions& options = ::grpc::StubOptions());

    class Service : public ::grpc::Service {
    public:
        Service();
        virtual ~Service();
        // Subscribe to 'transponder' updates.
        virtual ::grpc::Status SubscribeTransponder(
            ::grpc::ServerContext* context,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* request,
            ::grpc::ServerWriter<::mavsdk::rpc::transponder::TransponderResponse>* writer);
        // Set rate to 'transponder' updates.
        virtual ::grpc::Status SetRateTransponder(
            ::grpc::ServerContext* context,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest* request,
            ::mavsdk::rpc::transponder::SetRateTransponderResponse* response);
    };
    template<class BaseClass> class WithAsyncMethod_SubscribeTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithAsyncMethod_SubscribeTransponder() { ::grpc::Service::MarkMethodAsync(0); }
        ~WithAsyncMethod_SubscribeTransponder() override
        {
            BaseClassMustBeDerivedFromService(this);
        }
        // disable synchronous version of this method
        ::grpc::Status SubscribeTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* /*request*/,
            ::grpc::ServerWriter<::mavsdk::rpc::transponder::TransponderResponse>* /*writer*/)
            override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestSubscribeTransponder(
            ::grpc::ServerContext* context,
            ::mavsdk::rpc::transponder::SubscribeTransponderRequest* request,
            ::grpc::ServerAsyncWriter<::mavsdk::rpc::transponder::TransponderResponse>* writer,
            ::grpc::CompletionQueue* new_call_cq,
            ::grpc::ServerCompletionQueue* notification_cq,
            void* tag)
        {
            ::grpc::Service::RequestAsyncServerStreaming(
                0, context, request, writer, new_call_cq, notification_cq, tag);
        }
    };
    template<class BaseClass> class WithAsyncMethod_SetRateTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithAsyncMethod_SetRateTransponder() { ::grpc::Service::MarkMethodAsync(1); }
        ~WithAsyncMethod_SetRateTransponder() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status SetRateTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest* /*request*/,
            ::mavsdk::rpc::transponder::SetRateTransponderResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestSetRateTransponder(
            ::grpc::ServerContext* context,
            ::mavsdk::rpc::transponder::SetRateTransponderRequest* request,
            ::grpc::ServerAsyncResponseWriter<
                ::mavsdk::rpc::transponder::SetRateTransponderResponse>* response,
            ::grpc::CompletionQueue* new_call_cq,
            ::grpc::ServerCompletionQueue* notification_cq,
            void* tag)
        {
            ::grpc::Service::RequestAsyncUnary(
                1, context, request, response, new_call_cq, notification_cq, tag);
        }
    };
    typedef WithAsyncMethod_SubscribeTransponder<WithAsyncMethod_SetRateTransponder<Service>>
        AsyncService;
    template<class BaseClass> class WithCallbackMethod_SubscribeTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithCallbackMethod_SubscribeTransponder()
        {
            ::grpc::Service::MarkMethodCallback(
                0,
                new ::grpc::internal::CallbackServerStreamingHandler<
                    ::mavsdk::rpc::transponder::SubscribeTransponderRequest,
                    ::mavsdk::rpc::transponder::TransponderResponse>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* request) {
                        return this->SubscribeTransponder(context, request);
                    }));
        }
        ~WithCallbackMethod_SubscribeTransponder() override
        {
            BaseClassMustBeDerivedFromService(this);
        }
        // disable synchronous version of this method
        ::grpc::Status SubscribeTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* /*request*/,
            ::grpc::ServerWriter<::mavsdk::rpc::transponder::TransponderResponse>* /*writer*/)
            override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerWriteReactor<::mavsdk::rpc::transponder::TransponderResponse>*
        SubscribeTransponder(
            ::grpc::CallbackServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* /*request*/)
        {
            return nullptr;
        }
    };
    template<class BaseClass> class WithCallbackMethod_SetRateTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithCallbackMethod_SetRateTransponder()
        {
            ::grpc::Service::MarkMethodCallback(
                1,
                new ::grpc::internal::CallbackUnaryHandler<
                    ::mavsdk::rpc::transponder::SetRateTransponderRequest,
                    ::mavsdk::rpc::transponder::SetRateTransponderResponse>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::mavsdk::rpc::transponder::SetRateTransponderRequest* request,
                        ::mavsdk::rpc::transponder::SetRateTransponderResponse* response) {
                        return this->SetRateTransponder(context, request, response);
                    }));
        }
        void SetMessageAllocatorFor_SetRateTransponder(
            ::grpc::MessageAllocator<
                ::mavsdk::rpc::transponder::SetRateTransponderRequest,
                ::mavsdk::rpc::transponder::SetRateTransponderResponse>* allocator)
        {
            ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(1);
            static_cast<::grpc::internal::CallbackUnaryHandler<
                ::mavsdk::rpc::transponder::SetRateTransponderRequest,
                ::mavsdk::rpc::transponder::SetRateTransponderResponse>*>(handler)
                ->SetMessageAllocator(allocator);
        }
        ~WithCallbackMethod_SetRateTransponder() override
        {
            BaseClassMustBeDerivedFromService(this);
        }
        // disable synchronous version of this method
        ::grpc::Status SetRateTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest* /*request*/,
            ::mavsdk::rpc::transponder::SetRateTransponderResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerUnaryReactor* SetRateTransponder(
            ::grpc::CallbackServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest* /*request*/,
            ::mavsdk::rpc::transponder::SetRateTransponderResponse* /*response*/)
        {
            return nullptr;
        }
    };
    typedef WithCallbackMethod_SubscribeTransponder<WithCallbackMethod_SetRateTransponder<Service>>
        CallbackService;
    typedef CallbackService ExperimentalCallbackService;
    template<class BaseClass> class WithGenericMethod_SubscribeTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithGenericMethod_SubscribeTransponder() { ::grpc::Service::MarkMethodGeneric(0); }
        ~WithGenericMethod_SubscribeTransponder() override
        {
            BaseClassMustBeDerivedFromService(this);
        }
        // disable synchronous version of this method
        ::grpc::Status SubscribeTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* /*request*/,
            ::grpc::ServerWriter<::mavsdk::rpc::transponder::TransponderResponse>* /*writer*/)
            override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
    };
    template<class BaseClass> class WithGenericMethod_SetRateTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithGenericMethod_SetRateTransponder() { ::grpc::Service::MarkMethodGeneric(1); }
        ~WithGenericMethod_SetRateTransponder() override
        {
            BaseClassMustBeDerivedFromService(this);
        }
        // disable synchronous version of this method
        ::grpc::Status SetRateTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest* /*request*/,
            ::mavsdk::rpc::transponder::SetRateTransponderResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
    };
    template<class BaseClass> class WithRawMethod_SubscribeTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawMethod_SubscribeTransponder() { ::grpc::Service::MarkMethodRaw(0); }
        ~WithRawMethod_SubscribeTransponder() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status SubscribeTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* /*request*/,
            ::grpc::ServerWriter<::mavsdk::rpc::transponder::TransponderResponse>* /*writer*/)
            override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestSubscribeTransponder(
            ::grpc::ServerContext* context,
            ::grpc::ByteBuffer* request,
            ::grpc::ServerAsyncWriter<::grpc::ByteBuffer>* writer,
            ::grpc::CompletionQueue* new_call_cq,
            ::grpc::ServerCompletionQueue* notification_cq,
            void* tag)
        {
            ::grpc::Service::RequestAsyncServerStreaming(
                0, context, request, writer, new_call_cq, notification_cq, tag);
        }
    };
    template<class BaseClass> class WithRawMethod_SetRateTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawMethod_SetRateTransponder() { ::grpc::Service::MarkMethodRaw(1); }
        ~WithRawMethod_SetRateTransponder() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status SetRateTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest* /*request*/,
            ::mavsdk::rpc::transponder::SetRateTransponderResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestSetRateTransponder(
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
    template<class BaseClass> class WithRawCallbackMethod_SubscribeTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawCallbackMethod_SubscribeTransponder()
        {
            ::grpc::Service::MarkMethodRawCallback(
                0,
                new ::grpc::internal::
                    CallbackServerStreamingHandler<::grpc::ByteBuffer, ::grpc::ByteBuffer>(
                        [this](
                            ::grpc::CallbackServerContext* context,
                            const ::grpc::ByteBuffer* request) {
                            return this->SubscribeTransponder(context, request);
                        }));
        }
        ~WithRawCallbackMethod_SubscribeTransponder() override
        {
            BaseClassMustBeDerivedFromService(this);
        }
        // disable synchronous version of this method
        ::grpc::Status SubscribeTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* /*request*/,
            ::grpc::ServerWriter<::mavsdk::rpc::transponder::TransponderResponse>* /*writer*/)
            override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerWriteReactor<::grpc::ByteBuffer>* SubscribeTransponder(
            ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/)
        {
            return nullptr;
        }
    };
    template<class BaseClass> class WithRawCallbackMethod_SetRateTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawCallbackMethod_SetRateTransponder()
        {
            ::grpc::Service::MarkMethodRawCallback(
                1,
                new ::grpc::internal::CallbackUnaryHandler<::grpc::ByteBuffer, ::grpc::ByteBuffer>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::grpc::ByteBuffer* request,
                        ::grpc::ByteBuffer* response) {
                        return this->SetRateTransponder(context, request, response);
                    }));
        }
        ~WithRawCallbackMethod_SetRateTransponder() override
        {
            BaseClassMustBeDerivedFromService(this);
        }
        // disable synchronous version of this method
        ::grpc::Status SetRateTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest* /*request*/,
            ::mavsdk::rpc::transponder::SetRateTransponderResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerUnaryReactor* SetRateTransponder(
            ::grpc::CallbackServerContext* /*context*/,
            const ::grpc::ByteBuffer* /*request*/,
            ::grpc::ByteBuffer* /*response*/)
        {
            return nullptr;
        }
    };
    template<class BaseClass> class WithStreamedUnaryMethod_SetRateTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithStreamedUnaryMethod_SetRateTransponder()
        {
            ::grpc::Service::MarkMethodStreamed(
                1,
                new ::grpc::internal::StreamedUnaryHandler<
                    ::mavsdk::rpc::transponder::SetRateTransponderRequest,
                    ::mavsdk::rpc::transponder::SetRateTransponderResponse>(
                    [this](
                        ::grpc::ServerContext* context,
                        ::grpc::ServerUnaryStreamer<
                            ::mavsdk::rpc::transponder::SetRateTransponderRequest,
                            ::mavsdk::rpc::transponder::SetRateTransponderResponse>* streamer) {
                        return this->StreamedSetRateTransponder(context, streamer);
                    }));
        }
        ~WithStreamedUnaryMethod_SetRateTransponder() override
        {
            BaseClassMustBeDerivedFromService(this);
        }
        // disable regular version of this method
        ::grpc::Status SetRateTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SetRateTransponderRequest* /*request*/,
            ::mavsdk::rpc::transponder::SetRateTransponderResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        // replace default version of method with streamed unary
        virtual ::grpc::Status StreamedSetRateTransponder(
            ::grpc::ServerContext* context,
            ::grpc::ServerUnaryStreamer<
                ::mavsdk::rpc::transponder::SetRateTransponderRequest,
                ::mavsdk::rpc::transponder::SetRateTransponderResponse>* server_unary_streamer) = 0;
    };
    typedef WithStreamedUnaryMethod_SetRateTransponder<Service> StreamedUnaryService;
    template<class BaseClass>
    class WithSplitStreamingMethod_SubscribeTransponder : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithSplitStreamingMethod_SubscribeTransponder()
        {
            ::grpc::Service::MarkMethodStreamed(
                0,
                new ::grpc::internal::SplitServerStreamingHandler<
                    ::mavsdk::rpc::transponder::SubscribeTransponderRequest,
                    ::mavsdk::rpc::transponder::TransponderResponse>(
                    [this](
                        ::grpc::ServerContext* context,
                        ::grpc::ServerSplitStreamer<
                            ::mavsdk::rpc::transponder::SubscribeTransponderRequest,
                            ::mavsdk::rpc::transponder::TransponderResponse>* streamer) {
                        return this->StreamedSubscribeTransponder(context, streamer);
                    }));
        }
        ~WithSplitStreamingMethod_SubscribeTransponder() override
        {
            BaseClassMustBeDerivedFromService(this);
        }
        // disable regular version of this method
        ::grpc::Status SubscribeTransponder(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::transponder::SubscribeTransponderRequest* /*request*/,
            ::grpc::ServerWriter<::mavsdk::rpc::transponder::TransponderResponse>* /*writer*/)
            override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        // replace default version of method with split streamed
        virtual ::grpc::Status StreamedSubscribeTransponder(
            ::grpc::ServerContext* context,
            ::grpc::ServerSplitStreamer<
                ::mavsdk::rpc::transponder::SubscribeTransponderRequest,
                ::mavsdk::rpc::transponder::TransponderResponse>* server_split_streamer) = 0;
    };
    typedef WithSplitStreamingMethod_SubscribeTransponder<Service> SplitStreamedService;
    typedef WithSplitStreamingMethod_SubscribeTransponder<
        WithStreamedUnaryMethod_SetRateTransponder<Service>>
        StreamedService;
};

} // namespace transponder
} // namespace rpc
} // namespace mavsdk

#endif // GRPC_transponder_2ftransponder_2eproto__INCLUDED
