// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: tune/tune.proto
#ifndef GRPC_tune_2ftune_2eproto__INCLUDED
#define GRPC_tune_2ftune_2eproto__INCLUDED

#include "tune/tune.pb.h"

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
namespace tune {

// Enable creating and sending a tune to be played on the system.
class TuneService final {
public:
    static constexpr char const* service_full_name() { return "mavsdk.rpc.tune.TuneService"; }
    class StubInterface {
    public:
        virtual ~StubInterface() {}
        // Send a tune to be played by the system.
        virtual ::grpc::Status PlayTune(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest& request,
            ::mavsdk::rpc::tune::PlayTuneResponse* response) = 0;
        std::unique_ptr<
            ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::tune::PlayTuneResponse>>
        AsyncPlayTune(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::tune::PlayTuneResponse>>(
                AsyncPlayTuneRaw(context, request, cq));
        }
        std::unique_ptr<
            ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::tune::PlayTuneResponse>>
        PrepareAsyncPlayTune(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::tune::PlayTuneResponse>>(
                PrepareAsyncPlayTuneRaw(context, request, cq));
        }
        class async_interface {
        public:
            virtual ~async_interface() {}
            // Send a tune to be played by the system.
            virtual void PlayTune(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::tune::PlayTuneRequest* request,
                ::mavsdk::rpc::tune::PlayTuneResponse* response,
                std::function<void(::grpc::Status)>) = 0;
            virtual void PlayTune(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::tune::PlayTuneRequest* request,
                ::mavsdk::rpc::tune::PlayTuneResponse* response,
                ::grpc::ClientUnaryReactor* reactor) = 0;
        };
        typedef class async_interface experimental_async_interface;
        virtual class async_interface* async() { return nullptr; }
        class async_interface* experimental_async() { return async(); }

    private:
        virtual ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::tune::PlayTuneResponse>*
        AsyncPlayTuneRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
        virtual ::grpc::ClientAsyncResponseReaderInterface<::mavsdk::rpc::tune::PlayTuneResponse>*
        PrepareAsyncPlayTuneRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest& request,
            ::grpc::CompletionQueue* cq) = 0;
    };
    class Stub final : public StubInterface {
    public:
        Stub(
            const std::shared_ptr<::grpc::ChannelInterface>& channel,
            const ::grpc::StubOptions& options = ::grpc::StubOptions());
        ::grpc::Status PlayTune(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest& request,
            ::mavsdk::rpc::tune::PlayTuneResponse* response) override;
        std::unique_ptr<::grpc::ClientAsyncResponseReader<::mavsdk::rpc::tune::PlayTuneResponse>>
        AsyncPlayTune(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::tune::PlayTuneResponse>>(
                AsyncPlayTuneRaw(context, request, cq));
        }
        std::unique_ptr<::grpc::ClientAsyncResponseReader<::mavsdk::rpc::tune::PlayTuneResponse>>
        PrepareAsyncPlayTune(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest& request,
            ::grpc::CompletionQueue* cq)
        {
            return std::unique_ptr<
                ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::tune::PlayTuneResponse>>(
                PrepareAsyncPlayTuneRaw(context, request, cq));
        }
        class async final : public StubInterface::async_interface {
        public:
            void PlayTune(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::tune::PlayTuneRequest* request,
                ::mavsdk::rpc::tune::PlayTuneResponse* response,
                std::function<void(::grpc::Status)>) override;
            void PlayTune(
                ::grpc::ClientContext* context,
                const ::mavsdk::rpc::tune::PlayTuneRequest* request,
                ::mavsdk::rpc::tune::PlayTuneResponse* response,
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
        ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::tune::PlayTuneResponse>* AsyncPlayTuneRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest& request,
            ::grpc::CompletionQueue* cq) override;
        ::grpc::ClientAsyncResponseReader<::mavsdk::rpc::tune::PlayTuneResponse>*
        PrepareAsyncPlayTuneRaw(
            ::grpc::ClientContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest& request,
            ::grpc::CompletionQueue* cq) override;
        const ::grpc::internal::RpcMethod rpcmethod_PlayTune_;
    };
    static std::unique_ptr<Stub> NewStub(
        const std::shared_ptr<::grpc::ChannelInterface>& channel,
        const ::grpc::StubOptions& options = ::grpc::StubOptions());

    class Service : public ::grpc::Service {
    public:
        Service();
        virtual ~Service();
        // Send a tune to be played by the system.
        virtual ::grpc::Status PlayTune(
            ::grpc::ServerContext* context,
            const ::mavsdk::rpc::tune::PlayTuneRequest* request,
            ::mavsdk::rpc::tune::PlayTuneResponse* response);
    };
    template<class BaseClass> class WithAsyncMethod_PlayTune : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithAsyncMethod_PlayTune() { ::grpc::Service::MarkMethodAsync(0); }
        ~WithAsyncMethod_PlayTune() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status PlayTune(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::tune::PlayTuneRequest* /*request*/,
            ::mavsdk::rpc::tune::PlayTuneResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestPlayTune(
            ::grpc::ServerContext* context,
            ::mavsdk::rpc::tune::PlayTuneRequest* request,
            ::grpc::ServerAsyncResponseWriter<::mavsdk::rpc::tune::PlayTuneResponse>* response,
            ::grpc::CompletionQueue* new_call_cq,
            ::grpc::ServerCompletionQueue* notification_cq,
            void* tag)
        {
            ::grpc::Service::RequestAsyncUnary(
                0, context, request, response, new_call_cq, notification_cq, tag);
        }
    };
    typedef WithAsyncMethod_PlayTune<Service> AsyncService;
    template<class BaseClass> class WithCallbackMethod_PlayTune : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithCallbackMethod_PlayTune()
        {
            ::grpc::Service::MarkMethodCallback(
                0,
                new ::grpc::internal::CallbackUnaryHandler<
                    ::mavsdk::rpc::tune::PlayTuneRequest,
                    ::mavsdk::rpc::tune::PlayTuneResponse>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::mavsdk::rpc::tune::PlayTuneRequest* request,
                        ::mavsdk::rpc::tune::PlayTuneResponse* response) {
                        return this->PlayTune(context, request, response);
                    }));
        }
        void SetMessageAllocatorFor_PlayTune(::grpc::MessageAllocator<
                                             ::mavsdk::rpc::tune::PlayTuneRequest,
                                             ::mavsdk::rpc::tune::PlayTuneResponse>* allocator)
        {
            ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
            static_cast<::grpc::internal::CallbackUnaryHandler<
                ::mavsdk::rpc::tune::PlayTuneRequest,
                ::mavsdk::rpc::tune::PlayTuneResponse>*>(handler)
                ->SetMessageAllocator(allocator);
        }
        ~WithCallbackMethod_PlayTune() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status PlayTune(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::tune::PlayTuneRequest* /*request*/,
            ::mavsdk::rpc::tune::PlayTuneResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerUnaryReactor* PlayTune(
            ::grpc::CallbackServerContext* /*context*/,
            const ::mavsdk::rpc::tune::PlayTuneRequest* /*request*/,
            ::mavsdk::rpc::tune::PlayTuneResponse* /*response*/)
        {
            return nullptr;
        }
    };
    typedef WithCallbackMethod_PlayTune<Service> CallbackService;
    typedef CallbackService ExperimentalCallbackService;
    template<class BaseClass> class WithGenericMethod_PlayTune : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithGenericMethod_PlayTune() { ::grpc::Service::MarkMethodGeneric(0); }
        ~WithGenericMethod_PlayTune() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status PlayTune(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::tune::PlayTuneRequest* /*request*/,
            ::mavsdk::rpc::tune::PlayTuneResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
    };
    template<class BaseClass> class WithRawMethod_PlayTune : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawMethod_PlayTune() { ::grpc::Service::MarkMethodRaw(0); }
        ~WithRawMethod_PlayTune() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status PlayTune(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::tune::PlayTuneRequest* /*request*/,
            ::mavsdk::rpc::tune::PlayTuneResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        void RequestPlayTune(
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
    template<class BaseClass> class WithRawCallbackMethod_PlayTune : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithRawCallbackMethod_PlayTune()
        {
            ::grpc::Service::MarkMethodRawCallback(
                0,
                new ::grpc::internal::CallbackUnaryHandler<::grpc::ByteBuffer, ::grpc::ByteBuffer>(
                    [this](
                        ::grpc::CallbackServerContext* context,
                        const ::grpc::ByteBuffer* request,
                        ::grpc::ByteBuffer* response) {
                        return this->PlayTune(context, request, response);
                    }));
        }
        ~WithRawCallbackMethod_PlayTune() override { BaseClassMustBeDerivedFromService(this); }
        // disable synchronous version of this method
        ::grpc::Status PlayTune(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::tune::PlayTuneRequest* /*request*/,
            ::mavsdk::rpc::tune::PlayTuneResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        virtual ::grpc::ServerUnaryReactor* PlayTune(
            ::grpc::CallbackServerContext* /*context*/,
            const ::grpc::ByteBuffer* /*request*/,
            ::grpc::ByteBuffer* /*response*/)
        {
            return nullptr;
        }
    };
    template<class BaseClass> class WithStreamedUnaryMethod_PlayTune : public BaseClass {
    private:
        void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}

    public:
        WithStreamedUnaryMethod_PlayTune()
        {
            ::grpc::Service::MarkMethodStreamed(
                0,
                new ::grpc::internal::StreamedUnaryHandler<
                    ::mavsdk::rpc::tune::PlayTuneRequest,
                    ::mavsdk::rpc::tune::PlayTuneResponse>(
                    [this](
                        ::grpc::ServerContext* context,
                        ::grpc::ServerUnaryStreamer<
                            ::mavsdk::rpc::tune::PlayTuneRequest,
                            ::mavsdk::rpc::tune::PlayTuneResponse>* streamer) {
                        return this->StreamedPlayTune(context, streamer);
                    }));
        }
        ~WithStreamedUnaryMethod_PlayTune() override { BaseClassMustBeDerivedFromService(this); }
        // disable regular version of this method
        ::grpc::Status PlayTune(
            ::grpc::ServerContext* /*context*/,
            const ::mavsdk::rpc::tune::PlayTuneRequest* /*request*/,
            ::mavsdk::rpc::tune::PlayTuneResponse* /*response*/) override
        {
            abort();
            return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
        }
        // replace default version of method with streamed unary
        virtual ::grpc::Status StreamedPlayTune(
            ::grpc::ServerContext* context,
            ::grpc::ServerUnaryStreamer<
                ::mavsdk::rpc::tune::PlayTuneRequest,
                ::mavsdk::rpc::tune::PlayTuneResponse>* server_unary_streamer) = 0;
    };
    typedef WithStreamedUnaryMethod_PlayTune<Service> StreamedUnaryService;
    typedef Service SplitStreamedService;
    typedef WithStreamedUnaryMethod_PlayTune<Service> StreamedService;
};

} // namespace tune
} // namespace rpc
} // namespace mavsdk

#endif // GRPC_tune_2ftune_2eproto__INCLUDED
